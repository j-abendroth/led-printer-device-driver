# FreeBSD Character Device Driver
## John Abendroth, Ryan Tucker, Austin Seyboldt

### Build info
- Run 'make' and then 'make load' to build the kernel module. The device name will be added to devfs as gpio_led0.

### Usage:
- The default gpio pins are 23 for reading and 26 for writing
- Use the ledtextctl program (/tools/) to control and configure the device.
- Only lower case ASCII characters are supported by default. Attempted to write and wait to read other characters will cause the read to block indefinitely. To use other characters (including capitals) add them to the dictionary with `ledtextctl`

#### How to use:
Open the device with a typical call to `open()`. Specify nonblocking reads here with the `O_NONBLOCK` flag in `open()`. Then make typical `write()` and `read()` calls to the device.

#### ledtextctl tool usage:
- Change the min time between flashes in milliseconds (min time is 20ms):
    - ./ledtextctl -f /dev/gpio_led0 -t x_time
- Change the input or output pin
    - ./ledtextctl -f /dev/gpio_led0 -c pin_num (IN | OUT)
- Print the current dictionary
    - ./ledtextctl -f /dev/gpio_led0 -d get 
- Set a new dictionary
    - ./ledtextctl -f /dev/gpio_led0 -d set /path/to/dictionary/file
    - file must be a comma separated list of uint64_t values

### How does it work?
Writing to the device copies the text to be written to a kernel buffer, which adds a task to a taskqueue to flash the led for each character as defined in a huffman tree. These flashes then trigger interrupts from the gpiobus, which mesasures the intervals between flashes and traverses the huffman tree to decode the flashes. As characters are decoded, they're added to a read buffer. Once the read buffer hits a requested size, it signals it's ready to be read and the text is moved back to userspace, and the read buffer is moved forward to account for the data being read from it.

To see a full explanation of the design, see the design document in /docs/!

### Known bugs
- Once a user process has blocked for a read, it is impossible to send the process a kill signal. Do not block on a read if you do not expect data to become available because your pi will be stuck (at least in the current shell).
- There is no graceful way of handling device unloads in all situations. This is due to the locks used throughout our code. For example, if you try to unload the device mid-read, there is a high probablity the kernel will panic due to attempting to destroy a mutex while it is being waited on. We attempt to mitigate this by waking a blocked read thread before unloading, but if the ithread was still waiting the kernel will panic.

### Non-Blocking Reads
We implemented non-blocking reads a little different than what was specified on piazza, and the complete behavior is discussed in the design document under the reads section. In short, the driver only returns read data when the complete amount requested is ready, and will return `EAGAIN` if the read buffer doesn't have at least the requested amount available.
  
  
### Citations:
general stuff:  
- https://docs.freebsd.org/en/books/arch-handbook/driverbasics-char.html  
- /sys/dev/gpio/gpioled.c

Interrupt support taken from:
- /sys/dev/gpio/gpioc.c
- /sys/dev/gpio/gpiobus.c
- /sys/dev/gpio/gpiopps.c
- /sys/dev/gpio/gpiokeys.c
- /sys/dev/gpio/gpio_if.m

ioctl (how to return info to user):
- https://github.com/freebsd/freebsd-src/blob/releng/13.0/sys/geom/geom_dev.c

Taskqueue:
- https://gist.github.com/khanzf/465eec49d5008f460aa0571604267d7b

Autoconfig:
- https://docs.freebsd.org/en/books/arch-handbook/pci.html  

uimove functionality from freebsd.org