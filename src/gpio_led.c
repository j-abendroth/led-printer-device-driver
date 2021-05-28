// clang-format off
#include <sys/param.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/sx.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/condvar.h>
#include <sys/uio.h>
#include <sys/time.h>

#include <sys/buf_ring.h>
#include <sys/ioccom.h>
#include <sys/queue.h>
#include <sys/taskqueue.h>
#include <sys/fcntl.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/led/led.h>

#include <dev/gpio/gpiobusvar.h>

// clang-format on

/*
    TODOS:
        - refine ledtextctl (prettier formatting on GET_DICT)

    BUGS:
        - ctrl+c does not work if program is waiting for read data?
        - 'sudo kldunload' crashes kernel if we do it while waiting on a read (I think because
           of kernel panic due to destroying mutex while someone is waiting on it)
        -
 */

#define DEVICE_NAME       "gpio_led"
#define DEFAULT_X_TIME    20    // milliseconds
#define BUFFER_LEN        8192
#define MAX_QUEUED_WRITES 1000
#define DEFAULT_READ_PIN  23
#define DEFAULT_WRITE_PIN 26
#define DICT_SIZE         128
typedef struct led_ioctl_dict {
    uint64_t dict[DICT_SIZE];
} led_ioctl_data;

#define GET_DICTIONARY _IOR('L', 1, led_ioctl_data)
#define SET_DICTIONARY _IOW('L', 2, led_ioctl_data)
#define SET_INPUT_PIN  _IOW('L', 3, int)
#define SET_OUTPUT_PIN _IOW('L', 4, int)
#define SET_MIN_TIME   _IOW('L', 5, int)

typedef struct led_softc {
    struct cdev* led_cdev;
    device_t dev;        // this device
    device_t bus_dev;    // the bus
    gpio_pin_t read_pin;
    gpio_pin_t write_pin;
    struct buf_ring* br;      // holds write buffers
    struct taskqueue* tsq;    // pulls write tasks and executes
    struct task tsk;
    bool read_busy;
    int read_request_size;
    char* read_buf;
    int read_buf_length;
    int read_buf_index;    // Where to start reading from circular read buffer
    int write_index;       // Where to start copying into circular read buffer
    struct mtx buf_ring_mtx;
    struct mtx read_buf_mtx;
    struct mtx read_busy_lock;
    struct mtx huffman_lock;
    struct cv read_buf_ready;
    struct sx write_pin_lock;    // used to synchronize updating of pin with writer
    struct sx read_pin_lock;
    struct node* huffman_root;
    struct node* huffman_current;
    uint64_t x_time;
    // condition variable neeeded for waking reader who is blocked

    struct led_pin_intr* sc_read_pin_intr;
} led_softc;

/**
 * Struct to hold read pin values between interrupts
 *
 * Stores interrupt times and led values at each interrupt
 */
struct intr_pin_value {
    time_t sec;
    time_t recent_sec;
    suseconds_t micro_sec;
    suseconds_t recent_micro_sec;
    int led_value;
    int recent_value;
};

/**
 * Struct to hold resources for interrupt handler
 *
 * holds references to the interrupt handler resources and read pin
 * neede during interrupts
 */
struct led_pin_intr {
    struct led_softc* sc;
    gpio_pin_t read_pin;
    int intr_rid;
    struct resource* intr_res;
    void* intr_cookie;
    struct intr_pin_value* led;
};

static uint64_t dict[DICT_SIZE] = {
    0,           0,           0,           0,           0,           0,
    0,           0,           0,           0,           0,           0,
    0,           0,           0,           0,           0,           0,
    0,           0,           0,           0,           0,           0,
    0,           0,           0,           0,           0,           0,
    0,           0,           918304,      225970977,   0,           0,
    0,           0,           0,           0,           0,           0,
    0,           0,           2754092,     226364205,   13895470,    0,
    3618901808,  14476382513, 57906434866, 28952236595, 28952367668, 115812734005,
    57904993078, 57905124151, 28953023032, 28952629817, 1809321530,  226233147,
    0,           0,           0,           28051519,    0,           0,
    0,           0,           0,           0,           0,           0,
    0,           0,           0,           0,           0,           0,
    0,           0,           0,           0,           0,           0,
    0,           0,           0,           0,           0,           0,
    0,           0,           0,           0,           0,           904793439,
    0,           1311841,     2623074,     5768803,     3016036,     131941,
    5113446,     6817383,     525416,      787561,      225577834,   5769067,
    2360684,     7079533,     1049710,     918639,      3016304,     225446769,
    1138,        132211,      1574004,     7210613,     5900150,     5899895,
    112855672,   4982393,     452201594,   0,           0,           0,
    0,           0,
};

MALLOC_DECLARE(M_GPIO_LED);
MALLOC_DEFINE(M_GPIO_LED, DEVICE_NAME, "gpio_led device data");

static d_read_t led_read;
static d_write_t led_write;
static d_open_t led_open;
static d_close_t led_close;
static d_ioctl_t led_ioctl;

static void write_char(char c, led_softc* sc);
static void write_to_laser(void* arg, int pending);
void populate_write_dict(uint64_t* raw_dict);
static void decode_flash(led_softc* sc, int huffman_bit);

static int gpio_led_intrfilter(void*);
static void gpio_led_intrthread(void*);
static int gpio_led_detach(device_t dev);

static struct cdevsw led_cdevsw = {
    .d_version = D_VERSION,
    .d_open    = led_open,
    .d_close   = led_close,
    .d_read    = led_read,
    .d_write   = led_write,
    .d_ioctl   = led_ioctl,
    .d_name    = DEVICE_NAME,
};

// Huffman node declarations ========================================================================

struct node {
    struct node* left;
    struct node* right;
    char data;
};

static struct node* make_new_node(void);
static struct node* node_move_left(struct node* nd);
static struct node* node_move_right(struct node* nd);
static struct node* traverse_tree(struct node* nd, int val);
static struct node* construct_tree(uint64_t dict[], int len);
static void destroy_tree(struct node* root);
static bool is_huffman_leaf(struct node* nd);
static char get_huffman_node_char(struct node* nd);
// static void print_tree(struct node*);

// =================================================================================================

static int
dealloc_intr(struct led_softc* sc) {
    int err;
    if (sc->sc_read_pin_intr->intr_cookie != NULL) {
        // device_printf(sc->dev, "tearing down bus intr...\n");
        err = bus_teardown_intr(sc->sc_read_pin_intr->read_pin->dev, sc->sc_read_pin_intr->intr_res,
                                sc->sc_read_pin_intr->intr_cookie);
        if (err) {
            return (err);
        }
        sc->sc_read_pin_intr->intr_cookie = NULL;
    }

    if (sc->sc_read_pin_intr->intr_res != NULL) {
        // device_printf(sc->dev, "releasing bus resources\n");
        err = bus_release_resource(sc->sc_read_pin_intr->read_pin->dev, SYS_RES_IRQ, sc->sc_read_pin_intr->intr_rid,
                                   sc->sc_read_pin_intr->intr_res);
        if (err) {
            printf("error releasing resources\n");
            return (err);
        }
        sc->sc_read_pin_intr->intr_rid = 0;
        sc->sc_read_pin_intr->intr_res = NULL;
    }

    return (0);
}

static int
alloc_intr(struct led_softc* sc) {
    sc->sc_read_pin_intr->intr_res =
        gpio_alloc_intr_resource(sc->sc_read_pin_intr->read_pin->dev, &sc->sc_read_pin_intr->intr_rid, RF_ACTIVE,
                                 sc->read_pin, GPIO_INTR_EDGE_BOTH);
    if (sc->sc_read_pin_intr->intr_res == NULL) {
        // interrupt alloc failed, problems
        // device_printf(sc->dev, "device could not allocate interrupt resources\n");
        return (ENXIO);
    }

    int err = bus_setup_intr(sc->sc_read_pin_intr->read_pin->dev, sc->sc_read_pin_intr->intr_res, INTR_TYPE_MISC,
                             gpio_led_intrfilter, gpio_led_intrthread, sc->sc_read_pin_intr,
                             &sc->sc_read_pin_intr->intr_cookie);
    if (err != 0) {
        // device_printf(sc->dev, "Unable to set up interrupt handler\n");
        return (err);
    }
    // device_printf(sc->dev, "Interrupt handler successfully installed\n");

    return (0);
}

/**
 * Set up interrupt handler on default read pin
 *
 * Interrupt handling inspired heavily by:
 * /sys/dev/gpio/gpioc.c
 * /sys/dev/gpio/gpiobus.c
 *
 * use GPIO_INTR_EDGE_BOTH for intr on both pin rising and falling edge
 */
static int
setup_intr(struct led_softc* sc) {
    sc->sc_read_pin_intr                        = malloc(sizeof(struct led_pin_intr), M_GPIO_LED, M_WAITOK | M_ZERO);
    sc->sc_read_pin_intr->led                   = malloc(sizeof(struct intr_pin_value), M_GPIO_LED, M_WAITOK | M_ZERO);
    sc->sc_read_pin_intr->sc                    = sc;
    sc->sc_read_pin_intr->intr_res              = NULL;
    sc->sc_read_pin_intr->intr_cookie           = NULL;
    sc->sc_read_pin_intr->read_pin              = sc->read_pin;
    sc->sc_read_pin_intr->led->sec              = 0;
    sc->sc_read_pin_intr->led->recent_sec       = 0;
    sc->sc_read_pin_intr->led->micro_sec        = 0;
    sc->sc_read_pin_intr->led->recent_micro_sec = 0;
    sc->sc_read_pin_intr->led->led_value        = -1;
    sc->sc_read_pin_intr->led->recent_value     = -1;

    int err;
    if ((err = alloc_intr(sc)) != 0) {
        // gpio_led_detach(dev);
        return (err);
    }

    return (0);
}

static int
led_ioctl(struct cdev* dev, u_long cmd, caddr_t data, int flag, struct thread* td) {
    int error            = 0;
    struct led_softc* sc = dev->si_drv1;

    switch (cmd) {
        case GET_DICTIONARY: {
            led_ioctl_data* user_data = (led_ioctl_data*)data;
            for (int i = 0; i < DICT_SIZE; i++) {
                user_data->dict[i] = dict[i];
            }
            break;
        }
        case SET_DICTIONARY:
            sx_xlock(&sc->write_pin_lock);
            mtx_lock(&sc->huffman_lock);

            led_ioctl_data* user_dict = (led_ioctl_data*)data;
            populate_write_dict((uint64_t*)(&user_dict->dict));
            struct node* new_huff_tree = construct_tree(dict, DICT_SIZE);
            destroy_tree(sc->huffman_root);
            sc->huffman_root    = new_huff_tree;
            sc->huffman_current = sc->huffman_root;

            mtx_unlock(&sc->huffman_lock);
            sx_xunlock(&sc->write_pin_lock);
            break;
        case SET_OUTPUT_PIN:
            sx_xlock(&sc->write_pin_lock);
            int err = 0;

            int new_pin = *(int*)data;
            if (new_pin < 0 || new_pin > 40) {
                return EINVAL;
            }

            gpio_pin_t temp_pin = sc->write_pin;
            if ((err = gpio_pin_get_by_bus_pinnum(sc->bus_dev, new_pin, &(sc->write_pin)) != 0)) {
                // printf("failed to aquire pin\n");
                sx_xunlock(&sc->write_pin_lock);
                return (err);
            }
            if ((err = gpio_pin_setflags(sc->write_pin, GPIO_PIN_OUTPUT) != 0)) {
                // printf("failed to set pin flags\n");
                gpio_pin_release(sc->write_pin);
                sc->write_pin = temp_pin;
                sx_xunlock(&sc->write_pin_lock);
                return (err);
            }
            gpio_pin_release(temp_pin);
            sx_xunlock(&sc->write_pin_lock);
            break;
        case SET_INPUT_PIN: { /**
                               * Swap input pin to new pin number
                               *
                               * Currently set to call detach and destroy device on error here
                               * Idea is after interrupt handler is removed, can't easily encounter error and reset
                               * since there won't be any interrupts installed
                               * Thus have to complete everything successfully
                               *
                               * Possibly change approach in future
                               */

            int new_pin_num = *(uint32_t*)data;
            if (new_pin_num == sc->read_pin->pin) {
                break;
            }
            if (new_pin_num < 0 || new_pin_num > 40) {
                return EINVAL;
            }

            gpio_pin_t temp_pin = sc->read_pin;

            /* try acquiring new specified pin */
            if ((error = gpio_pin_get_by_bus_pinnum(sc->bus_dev, new_pin_num, &(sc->read_pin)) != 0)) {
                // device_printf(sc->dev, "failed to aquire read pin\n");
                sc->read_pin = temp_pin;
                return (error);
            }
            device_printf(sc->dev, "acquired read pin: %d\n", new_pin_num);
            if ((error = gpio_pin_setflags(sc->read_pin, GPIO_PIN_INPUT) != 0)) {
                // device_printf(sc->dev, "failed to set read pin flags\n");
                gpio_pin_release(sc->read_pin);
                sc->read_pin = temp_pin;
                return (error);
            }

            /* successfully got new pin, dealloc old interrupt resources */
            if ((error = dealloc_intr(sc)) != 0) {
                // printf("Error dealloc intr\n");
                return (error);
            }

            gpio_pin_release(temp_pin);

            /* update interrupt resource struct with new pin pointer and install the new interrupt */
            sc->sc_read_pin_intr->read_pin = sc->read_pin;

            if ((error = alloc_intr(sc)) != 0) {
                return (error);
            }

            break;
        }
        case SET_MIN_TIME: {
            int new_time = *(int*)data;
            if (new_time < 20) {
                return EINVAL;
            }
            sx_xlock(&sc->write_pin_lock);
            sc->x_time = (uint64_t)new_time;
            sx_xunlock(&sc->write_pin_lock);
            break;
        }
        default: break;
    }
    return (error);
}

/* uiomove functionality sourcef from echo example on freebsd.org */
static int
led_read(struct cdev* dev, struct uio* uio, int ioflag) {
    int bytes_to_read = MIN(uio->uio_resid, BUFFER_LEN);
    if (bytes_to_read == 0) {
        return 0;
    }
    int error = 0;
    bool block;
    struct led_softc* sc;
    sc = dev->si_drv1;

    if ((O_NONBLOCK & ioflag) == O_NONBLOCK) {
        // printf("non block read\n");
        block = false;
    } else {
        // printf("blocking read\n");
        block = true;
    }

    // check if we can read
    mtx_lock(&sc->read_busy_lock);
    if (sc->read_busy) {
        mtx_unlock(&sc->read_busy_lock);
        return EBUSY;
    } else {
        sc->read_busy = true;
        mtx_unlock(&sc->read_busy_lock);
    }

    // wait until the buffer has the requested number of bytes
    mtx_lock(&sc->read_buf_mtx);
    sc->read_request_size = bytes_to_read;
    while (sc->read_buf_length < bytes_to_read) {
        if (!block) {
            // if nonblocking and don't have bytes ready, return EAGAIN
            sc->read_request_size = 0;
            sc->read_busy         = false;
            mtx_unlock(&sc->read_buf_mtx);
            return (EAGAIN);
        }
        cv_wait(&sc->read_buf_ready, &sc->read_buf_mtx);
    }

    sc->read_buf[sc->read_buf_index + sc->read_buf_length] = '\0';
    // copy the characters into user space
    while (uio->uio_resid > 0) {
        int bytes = MIN(uio->uio_resid, BUFFER_LEN - sc->read_buf_index);
        if ((error = uiomove(&sc->read_buf[sc->read_buf_index], bytes, uio)) != 0) {
            sc->read_request_size = 0;
            sc->read_busy         = false;
            mtx_unlock(&sc->read_buf_mtx);
            return (error);
        }
        sc->read_buf_length -= bytes;
        sc->read_buf_index = (sc->read_buf_index + bytes) % BUFFER_LEN;
    }
    sc->read_request_size = 0;
    sc->read_busy         = false;
    mtx_unlock(&sc->read_buf_mtx);
    return error;
}

/* uiomove functionality sourced from echo example on freebsd.org */
static int
led_write(struct cdev* dev, struct uio* uio, int ioflag __unused) {
    // printf("led_write called\n");
    int error = 0;
    struct led_softc* sc;
    char* write_buf;
    size_t amt;

    sc  = dev->si_drv1;
    amt = MIN(uio->uio_resid, BUFFER_LEN);

    // allocate write buffer and copy user data into it
    write_buf = malloc(sizeof(char) * (amt + 1), M_GPIO_LED, M_WAITOK | M_ZERO);
    if ((error = uiomove(write_buf, amt, uio)) != 0) {
        // printf("Error uiomove\n");
        free(write_buf, M_GPIO_LED);
        return (error);
    }
    write_buf[uio->uio_offset] = '\0';

    // add the write buffer to the buf ring
    // printf("buf_ring slots: %d\n", buf_ring_count(sc->br));
    mtx_lock(&sc->buf_ring_mtx);
    if ((error = buf_ring_enqueue(sc->br, write_buf)) == ENOBUFS) {
        mtx_unlock(&sc->buf_ring_mtx);
        // printf("error buf_ring enqueue\n");
        free(write_buf, M_GPIO_LED);
        return (error);
    }
    mtx_unlock(&sc->buf_ring_mtx);
    // printf("buf_ring enqueue called\n");

    // add a task to the taskqueue to handle the write
    if (taskqueue_enqueue(taskqueue_swi, &(sc->tsk)) != 0) {
        // printf("taskqueue_enqueue error\n");
        free(write_buf, M_GPIO_LED);
    }
    // printf("taskqueue_enqueue called\n");

    return (error);
}

static int
led_open(struct cdev* dev, int oflags, int devtype __unused, struct thread* td __unused) {
    // do nothing?
    return 0;
}

static int
led_close(struct cdev* dev, int fflag __unused, int devtype __unused, struct thread* td __unused) {
    // printf("gpio_led closed\n");
    // do nothing?
    return 0;
}

static void
gpio_led_identify(driver_t* driver, device_t parent) {
    // printf("gpio_led_identify called\n");
    device_t child;
    child = device_find_child(parent, DEVICE_NAME, -1);
    if (!child) {
        child = BUS_ADD_CHILD(parent, 0, DEVICE_NAME, -1);
    }
    // printf("gpio_led_identify finished\n");
}

// code sourced from /dev/gpio/gpioled.c
static int
gpio_led_probe(device_t dev) {
    // device_printf(dev, "gpio_led being probed\n");

    device_set_desc(dev, "GPIO led printer");

    return (BUS_PROBE_DEFAULT);
}

static int
gpio_led_attach(device_t dev) {
    // device_printf(dev, "in gpio_led_attach\n");
    int err = 0;
    struct led_softc* sc;
    // initialize everything to NULL in case we fail somewhere -> no invalid pointers
    sc                    = device_get_softc(dev);
    sc->dev               = dev;
    sc->bus_dev           = device_get_parent(dev);
    sc->led_cdev          = NULL;
    sc->br                = NULL;
    sc->tsq               = NULL;
    sc->read_buf          = NULL;
    sc->x_time            = DEFAULT_X_TIME;
    sc->huffman_root      = NULL;
    sc->huffman_current   = NULL;
    sc->read_busy         = false;
    sc->read_buf_index    = 0;
    sc->read_buf_length   = 0;
    sc->read_request_size = 0;

    // acquire read and write pins
    if ((err = gpio_pin_get_by_bus_pinnum(sc->bus_dev, DEFAULT_WRITE_PIN, &(sc->write_pin)) != 0)) {
        // device_printf(dev, "failed to aquire write pin\n");
        // gpio_led_detach(dev);
        return (err);
    }
    if ((err = gpio_pin_setflags(sc->write_pin, GPIO_PIN_OUTPUT) != 0)) {
        // device_printf(dev, "failed to set pin flags\n");
        // gpio_led_detach(dev);
        return (err);
    }
    if ((err = gpio_pin_get_by_bus_pinnum(sc->bus_dev, DEFAULT_READ_PIN, &(sc->read_pin)) != 0)) {
        // device_printf(dev, "failed to aquire read pin\n");
        // gpio_led_detach(dev);
        return (err);
    }
    // device_printf(dev, "acquired read pin: %d\n", DEFAULT_READ_PIN);
    if ((err = gpio_pin_setflags(sc->read_pin, GPIO_PIN_INPUT) != 0)) {
        // device_printf(dev, "failed to set read pin flags\n");
        // gpio_led_detach(dev);
        return (err);
    }

    /* setup interrupts */
    if ((err = setup_intr(sc)) != 0) {
        // error setting up interrupts
        return (err);
    }

    /* allocate buffers and mutexes */
    cv_init(&sc->read_buf_ready, "read_buf cond_var");
    mtx_init(&sc->huffman_lock, "huffman tree lock", NULL, MTX_DEF);
    mtx_init(&sc->read_busy_lock, "synchronous read lock", NULL, MTX_DEF);
    mtx_init(&sc->buf_ring_mtx, "buf_ring_mtx_gpio_led", NULL, MTX_DEF);
    mtx_init(&sc->read_buf_mtx, "read_buffer lock", NULL, MTX_DEF);
    sx_init(&sc->write_pin_lock, "write_pin lock");
    sx_init(&sc->read_pin_lock, "read_pin lock");
    sc->read_buf = malloc(sizeof(char) * BUFFER_LEN, M_GPIO_LED, M_WAITOK | M_ZERO);

    // initialize a buf ring for write buffers and init task queue function
    sc->br = buf_ring_alloc(MAX_QUEUED_WRITES, M_DEVBUF, M_WAITOK, &(sc->buf_ring_mtx));
    TASK_INIT(&sc->tsk, 0, write_to_laser, sc);

    /* construct the huffman tree */
    sc->huffman_root    = construct_tree(dict, DICT_SIZE);
    sc->huffman_current = sc->huffman_root;
    // device_printf(dev, "created huffman tree\n");

    // make the device
    // make_dev_s functionality sourced from: /sys/dev/gpio/gpiopps.c
    struct make_dev_args devargs;
    make_dev_args_init(&devargs);
    devargs.mda_devsw   = &led_cdevsw;
    devargs.mda_uid     = UID_ROOT;
    devargs.mda_gid     = GID_WHEEL;
    devargs.mda_mode    = 0660;
    devargs.mda_si_drv1 = sc;
    err                 = make_dev_s(&devargs, &sc->led_cdev, DEVICE_NAME "%d", device_get_unit(dev));
    if (err != 0) {
        // device_printf(dev, "Unable to create gpio_led cdev\n");
        // gpio_led_detach(dev);
        return (err);
    } else {
        // device_printf(dev, "device attached\n");
    }

    return (0);
}

static int
gpio_led_detach(device_t dev) {
    // device_printf(dev, "in gpio_led detach\n");
    struct led_softc* sc = device_get_softc(dev);

    // destroy the huffman tree
    if (sc->huffman_root) {
        destroy_tree(sc->huffman_root);
        sc->huffman_root    = NULL;
        sc->huffman_current = NULL;
    }

    // destroy read buffer and release all pins
    if (sc->read_buf) {
        free(sc->read_buf, M_GPIO_LED);
        sc->read_buf = NULL;
    }
    if (sc->write_pin)
        gpio_pin_release(sc->write_pin);

    if (sc->read_pin)
        gpio_pin_release(sc->read_pin);

    // clear the buf ring and destroy it
    char* string;
    if (sc->br) {
        while (!buf_ring_empty(sc->br)) {
            string = buf_ring_dequeue_mc(sc->br);
            // device_printf(dev, "Here's an excess buffer: %s\n", string);
            free(string, M_GPIO_LED);
        }
        buf_ring_free(sc->br, M_DEVBUF);
        sc->br = NULL;
    }

    if (sc->read_buf) {
        free(sc->read_buf, M_GPIO_LED);
        sc->read_buf = NULL;
    }

    // no other threads can be holding a lock when we destroy them!!
    // signal any waiting reader to awaken and then destroy all mutexes
    cv_signal(&sc->read_buf_ready);
    cv_destroy(&sc->read_buf_ready);
    mtx_destroy(&sc->read_busy_lock);
    mtx_destroy(&sc->buf_ring_mtx);
    mtx_destroy(&sc->read_buf_mtx);
    mtx_destroy(&sc->huffman_lock);
    sx_destroy(&sc->write_pin_lock);
    sx_destroy(&sc->read_pin_lock);

    /* Tear down interrupt handler */
    dealloc_intr(sc);

    // free pointers
    if (sc->sc_read_pin_intr->led) {
        // device_printf(dev, "freeing read pin\n");
        free(sc->sc_read_pin_intr->led, M_GPIO_LED);
    }

    if (sc->sc_read_pin_intr) {
        // device_printf(dev, "freeing read pin intr\n");
        free(sc->sc_read_pin_intr, M_GPIO_LED);
    }

    if (sc->led_cdev) {
        // device_printf(dev, "destroying device...\n");
        destroy_dev_sched(sc->led_cdev);
    }
    // printf("device detached\n");

    return 0;
}

/**
 * Filter interrupt handler
 *
 * Collects hardware interrupt immediately and schedules ithread to run later
 * Saves the current interrupt time and pin state for ithread to process
 */
static int
gpio_led_intrfilter(void* arg) {
    struct led_pin_intr* intr_conf;
    struct timeval cur_time;

    intr_conf = arg;

    /* get pin value and current time, then ack interrupt and schedule ithread */

    microuptime(&cur_time);
    intr_conf->led->sec       = cur_time.tv_sec;
    intr_conf->led->micro_sec = cur_time.tv_usec;

    /* get value of read pin from gpio_if.m KOBJ interface */
    bool onoff = false;
    if (gpio_pin_is_active(intr_conf->read_pin, &onoff) != 0) {
        // error getting pin value
        device_printf(intr_conf->sc->dev, "Error getting photoresistor pin value. Will default to logical value 0\n");
    }
    intr_conf->led->led_value = onoff ? 1 : 0;
    if (intr_conf->led->led_value == intr_conf->led->recent_value)
        return (FILTER_HANDLED);

    return (FILTER_SCHEDULE_THREAD);
}

/**
 * Interrupt handler ithread
 * Runs after filter to process the interrupt interval
 * Adds appropriate bit to huffman encoding tree traversal
 *
 * If decode flash finds a leaf in huffman tree, adds character to read buffer
 */
static void
gpio_led_intrthread(void* arg) {
    struct led_pin_intr* intr_conf;
    int huffman_bit;
    uint32_t current_ms, previous_ms, interval;

    intr_conf            = arg;
    struct led_softc* sc = intr_conf->sc;
    huffman_bit          = -1;

    /* If first interrupt, no processing to do so shift current time to recent time */
    if (intr_conf->led->recent_sec == 0 && intr_conf->led->recent_micro_sec == 0) {
        intr_conf->led->recent_sec       = intr_conf->led->sec;
        intr_conf->led->recent_micro_sec = intr_conf->led->micro_sec;
        intr_conf->led->recent_value     = intr_conf->led->led_value;
        intr_conf->led->led_value        = -1;

        return;
    }

    /**
     * Calculate interval between last interrupt and the current one
     *
     * Possibly susceptible to int overflow since multiplying seconds * 1000
     * Relying on system having been rebooted within last ~50 days
     */
    current_ms  = (intr_conf->led->micro_sec / 1000) + (intr_conf->led->sec * 1000);
    previous_ms = (intr_conf->led->recent_micro_sec / 1000) + (intr_conf->led->recent_sec * 1000);
    interval    = current_ms - previous_ms;

    if (intr_conf->led->led_value == 0) {
        /* falling edge, just finished dot/dash */
        if (interval >= (sc->x_time / 2) && interval <= (sc->x_time * (3 / 2))) {
            // dot
            huffman_bit = 0;
        } else if (interval >= 3 * sc->x_time) {
            // dash
            huffman_bit = 1;
        }
        decode_flash(sc, huffman_bit);
    } else if (intr_conf->led->led_value == 1) {
        /* starting new dot or dash */
        if (interval < sc->x_time) {
            // problem
            printf("Warning: Minimum time between flashes not met. Characters may not be decoded properly\n");
        }
    }

    /* shift current interrupt time to previous for next interrupt */
    intr_conf->led->recent_sec       = intr_conf->led->sec;
    intr_conf->led->recent_micro_sec = intr_conf->led->micro_sec;
    intr_conf->led->sec              = 0;
    intr_conf->led->micro_sec        = 0;
    intr_conf->led->recent_value     = intr_conf->led->led_value;
    intr_conf->led->led_value        = -1;

    return;
}

/* This function is registered with the task struct in our cdev.
 * In other words, this is the function that is exec
 */
static void
write_to_laser(void* arg, int pending) {
    struct led_softc* sc = (struct led_softc*)arg;
    char* write_buf;

    // grab the write buf
    mtx_lock(&sc->buf_ring_mtx);
    write_buf = buf_ring_dequeue_mc(sc->br);
    mtx_unlock(&sc->buf_ring_mtx);

    // write the message character-by-character
    sx_xlock(&sc->write_pin_lock);
    for (int i = 0; write_buf[i] != '\0'; i++) {
        write_char(write_buf[i], sc);
    }
    sx_xunlock(&sc->write_pin_lock);

    // need to destory write buf!!
    free(write_buf, M_GPIO_LED);
}

static void
write_char(char c, led_softc* sc) {
    /* Encoding: uint64_t
        bits 0-7: the character
        bits 8-16: the length
        bits 17-64: the code
    */
    uint64_t encoding = dict[(int)c];
    uint64_t length   = (encoding >> 8) & 0xFF;
    uint64_t code     = encoding >> 17;

    // loop thru the bits in the code and flash them
    for (int i = (int)length - 1; i >= 0; i--) {
        uint64_t bit = (code >> i) & 1;

        if (bit == 0) {
            gpio_pin_set_active(sc->write_pin, true);
            tsleep_sbt(sc, PRIBIO, "gpio_led_write_sleep", (SBT_1MS * ((sc->x_time / 2) + 5)), 0, 0);
            gpio_pin_set_active(sc->write_pin, false);
        } else {
            gpio_pin_set_active(sc->write_pin, true);
            tsleep_sbt(sc, PRIBIO, "gpio_led_write_sleep", (SBT_1MS * ((sc->x_time * 3) + 5)), 0, 0);
            gpio_pin_set_active(sc->write_pin, false);
        }
        tsleep_sbt(sc, PRIBIO, "gpio_led_write_sleep", (SBT_1MS * (sc->x_time + 5)), 0, 0);
    }
}

/*
Translate a flash into ASCII.
Descends the huffman tree depending on if the flash is a 0 or 1.
When it hits a leaf node, it adds the character to the read buffer.
*/
static void
decode_flash(led_softc* sc, int huffman_bit) {
    // travel down the tree
    mtx_lock(&sc->huffman_lock);
    sc->huffman_current = traverse_tree(sc->huffman_current, huffman_bit);
    if (sc->huffman_current == NULL) {
        // something went wrong, we fell off the tree from an internal node
        sc->huffman_current = sc->huffman_root;
        mtx_unlock(&sc->huffman_lock);
        printf("ERROR: traverse_tree returned NULL\n");
    } else if (is_huffman_leaf(sc->huffman_current)) {
        // Its a leaf node, get the character
        char ch             = get_huffman_node_char(sc->huffman_current);
        sc->huffman_current = sc->huffman_root;
        mtx_unlock(&sc->huffman_lock);
        // printf("%c", ch);

        mtx_lock(&sc->read_buf_mtx);
        // read buffer is a circular buffer: find the location to place the new character
        int index_to_place           = (sc->read_buf_index + sc->read_buf_length) % BUFFER_LEN;
        sc->read_buf[index_to_place] = ch;
        if (sc->read_buf_length < BUFFER_LEN) {
            sc->read_buf_length++;
        }
        // if the buffer is full, overwrite the oldest value and advance the start index
        if (index_to_place == sc->read_buf_index && sc->read_buf_length == BUFFER_LEN) {
            sc->read_buf_index = ++sc->read_buf_index % BUFFER_LEN;
        }
        // if the buffer is the requested size, signal the waiting reader to wake up
        if (sc->read_buf_length >= sc->read_request_size && sc->read_request_size != 0) {
            cv_signal(&sc->read_buf_ready);
        }
        mtx_unlock(&sc->read_buf_mtx);
    } else {
        mtx_unlock(&sc->huffman_lock);
    }
}

void
populate_write_dict(uint64_t* raw_dict) {
    // first reset dictionary
    for (int i = 0; i < DICT_SIZE; i++) {
        dict[i] = 0;
    }

    // then populate it: important => expects the input dictionary to be 0-terminated
    for (int i = 0; raw_dict[i] != 0; i++) {
        uint64_t character = raw_dict[i] & 0xFF;
        if (character >= DICT_SIZE) {
            printf("error translating character\n");
        } else {
            dict[character] = raw_dict[i];
        }
    }
}

devclass_t gpio_led_devclass;

static device_method_t gpio_led_methods[] = {
    DEVMETHOD(device_identify, gpio_led_identify),
    DEVMETHOD(device_probe, gpio_led_probe),
    DEVMETHOD(device_attach, gpio_led_attach),
    DEVMETHOD(device_detach, gpio_led_detach),

    DEVMETHOD_END,

};

driver_t gpio_led_driver = {
    DEVICE_NAME,
    gpio_led_methods,
    sizeof(led_softc),
};

DRIVER_MODULE(gpio_led, gpiobus, gpio_led_driver, gpio_led_devclass, 0, 0);
MODULE_VERSION(gpio_led, 1);

// Node struct definitions =====================================================================

static struct node*
make_new_node() {
    struct node* new_node = malloc(sizeof(struct node), M_GPIO_LED, M_WAITOK | M_ZERO);
    new_node->data        = '\0';
    new_node->left        = NULL;
    new_node->right       = NULL;
    return new_node;
}

// static void
// print_tree(struct node* root) {
//     if (root == NULL) {
//         return;
//     }
//     print_tree(root->left);
//     printf("%c ", root->data);
//     print_tree(root->right);
// }

static void
destroy_tree(struct node* root) {
    if (root == NULL) {
        return;
    }
    destroy_tree(root->left);
    destroy_tree(root->right);
    free(root, M_GPIO_LED);
}

static struct node*
node_move_left(struct node* nd) {
    if (nd != NULL & nd->left != NULL) {
        return nd->left;
    }
    return NULL;
}

static struct node*
node_move_right(struct node* nd) {
    if (nd != NULL && nd->right != NULL) {
        return nd->right;
    }
    return NULL;
}

static struct node*
traverse_tree(struct node* nd, int val) {
    if (val == 0) {
        return node_move_left(nd);
    } else if (val == 1) {
        return node_move_right(nd);
    } else {
        return NULL;
    }
}

static bool
is_huffman_leaf(struct node* nd) {
    if (nd->right == NULL && nd->left == NULL) {
        return true;
    }
    return false;
}

static char
get_huffman_node_char(struct node* nd) {
    return nd->data;
}

struct node*
construct_tree(uint64_t dict[], int dict_len) {
    // 	for each item in dict:
    // 		extract code, length, char from item
    //		descend tree to proper node
    //		if its the last bit in code:
    //			set node->char = char

    struct node* root = make_new_node();

    for (int i = 0; i < dict_len; i++) {
        uint64_t encoding = dict[i];
        // continue if the entry is empty
        if (encoding == 0) {
            continue;
        }

        char character  = (char)(encoding & 0xFF);
        uint64_t length = (encoding >> 8) & 0xFF;
        uint64_t code   = encoding >> 17;

        // descend from the root
        struct node* cur = root;
        for (int i = (int)length - 1; i >= 0; i--) {
            uint64_t bit = (code >> i) & 1;

            if (bit == 0) {
                if (cur->left == NULL) {
                    cur->left = make_new_node();
                }
                cur = cur->left;
            } else {
                if (cur->right == NULL) {
                    cur->right = make_new_node();
                }
                cur = cur->right;
            }
            if (i == 0) {
                cur->data = (char)character;
            }
        }
    }
    return root;
}

// =============================================================================================

/**
 * CODE SOURCES:
 *
 * /sys/dev/gpio/gpioled.c
 *
 * Interrupt support taken from:
 * /sys/dev/gpio/gpioc.c
 * /sys/dev/gpio/gpiobus.c
 * /sys/dev/gpio/gpiopps.c
 * /sys/dev/gpio/gpiokeys.c
 * /sys/dev/gpio/gpio_if.m
 *
 */
