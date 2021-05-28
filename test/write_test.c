#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int
main() {
    int fd, nonblock_fd;
    const char* str = "hello there! -- general kenobi! you are a bold one!";

    nonblock_fd = open("/dev/gpio_led0", O_RDWR | O_NONBLOCK);
    if (nonblock_fd == -1) {
        printf("error opening file\n");
        exit(-1);
    }

    char buf2[strlen(str) + 1];
    buf2[strlen(str)] = '\0';
    int nonblock_read   = read(nonblock_fd, buf2, 51);
    if (nonblock_read == -1) {
        perror("nonblocking read returned -1");
    }
    printf("Read %d bytes from device: %s\n", nonblock_read, buf2);

    close(nonblock_fd);

    fd = open("/dev/gpio_led0", O_RDWR);
    if (fd == -1) {
        printf("error opening file\n");
        exit(-1);
    }
    
    printf("Writing this to device: %s\n", str);
    if (write(fd, str, strlen(str)) == -1) {
        perror("write fail");
    }

    char buf[strlen(str) + 1];
    buf[strlen(str)] = '\0';
    int bytes_read   = read(fd, buf, strlen(str));
    if (bytes_read == -1) {
        perror("read fail");
    }
    printf("Read %d bytes from device: %s\n", bytes_read, buf);

    close(fd);

    return (0);
}
