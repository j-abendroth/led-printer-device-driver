#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include "../src/led_ioctl.h"

int
test_set_pin(int fd) {
    printf("testing set output pin\n");
    int pin = 26;
    if (ioctl(fd, SET_OUTPUT_PIN, &pin) == -1) {
        perror("IOCTL ERROR");
        return 1;
    }
    return 0;
}

int
main() {
    int fd;
    if ((fd = open("/dev/gpio_led0", O_RDWR)) == -1) {
        printf("can't connect to gpio_led0\n");
        return 1;
    }

    test_set_pin(fd);
    close(fd);

    return 0;
}