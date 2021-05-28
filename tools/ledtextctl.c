/* SPECIFICATIONS:
    SET_INPUT_PIN/SET_OUTPUT_PIN
    ledtextctl -f /dev/ledtextN -c <pin> IN
    ledtextctl -f /dev/ledtextN -c <pin> OUT

    SET_MIN_TIME
    ledtextctl -f /dev/ledtextN -t <time in ms>

    GET_DICTIONARY
    ledtextctl -f /dev/ledtextN -d get

    SET_DICTIONARY
    ledtextctl -f /dev/ledtextN -d set <textfilename>
*/
#include "led_ioctl.h"
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define FILE_BUF_LEN 1024

extern int getopt(int argc, char* const argv[], const char* optstring);
extern char* optarg;
extern int optind;

int read_dict(char* file, led_ioctl_data*);

void
usage() {
    char usage_msg[] = "Usage:\n \
                      ledtextctl -f /path/to/device \
                      {-c <pin num>} {-t <time in ms>} \
                      {-d (get | set /path/to/dictionary)}";
    printf("%s\n", usage_msg);
}

void
cleanup(int device_fd, int dict_fd) {
    if (device_fd != -1) {
        close(device_fd);
    }
    if (dict_fd != -1) {
        close(dict_fd);
    }
}

int
main(int argc, char* argv[]) {
    char optstring[] = "f:c:t:d:";
    int opt;
    int device_fd = -1;
    int dict_fd   = -1;
    int pin_num   = -1;
    int time      = -1;

    while ((opt = getopt(argc, argv, optstring)) != -1) {
        switch (opt) {
            case 'f':
                // open the device
                if ((device_fd = open(optarg, O_RDWR, 0)) < 0) {
                    fprintf(stderr, "Could not open device: %s\n", strerror(errno));
                    cleanup(device_fd, dict_fd);
                    exit(EXIT_FAILURE);
                }
                break;
            case 't':
                // set the new time
                time = (int)strtol(optarg, NULL, 10);
                if (time <= 0) {
                    fprintf(stderr, "time value must be an integer greater than 0\n");
                    cleanup(device_fd, dict_fd);
                    exit(EXIT_FAILURE);
                } else if (ioctl(device_fd, SET_MIN_TIME, &time)) {
                    perror("SET_MIN_TIME");
                    cleanup(device_fd, dict_fd);
                    exit(EXIT_FAILURE);
                }
                break;
            case 'c':
                // change a pin
                pin_num = (int)strtol(optarg, NULL, 10);
                if (optind >= argc) {
                    fprintf(stderr, "pin option required\n");
                    cleanup(device_fd, dict_fd);
                    exit(EXIT_FAILURE);
                }
                if (strcmp(argv[optind], "IN") == 0) {
                    if (ioctl(device_fd, SET_INPUT_PIN, &pin_num) == -1) {
                        perror("SET_INPUT_PIN");
                        cleanup(device_fd, dict_fd);
                        exit(EXIT_FAILURE);
                    }
                } else if (strcmp(argv[optind], "OUT") == 0) {
                    if (ioctl(device_fd, SET_OUTPUT_PIN, &pin_num)) {
                        perror("SET_OUTPUT_PIN");
                        cleanup(device_fd, dict_fd);
                        exit(EXIT_FAILURE);
                    }
                } else {
                    fprintf(stderr, "invalid pin option\n");
                    cleanup(device_fd, dict_fd);
                    exit(EXIT_FAILURE);
                }
                optind++;
                break;
            case 'd': {
                led_ioctl_data data;
                for (int i = 0; i < 128; i++) {
                    data.dict[i] = 0;
                }
                if (strcmp(optarg, "get") == 0) {
                    if (ioctl(device_fd, GET_DICTIONARY, &data) == -1) {
                        perror("GET_DICTIONARY");
                        cleanup(device_fd, dict_fd);
                        exit(EXIT_FAILURE);
                    }
                    // print the dictionary
                    printf("\nCurrent Dictionary:\n");
                    for (int i = 0; i < DICT_SIZE; i++) {
                        if (data.dict[i] != 0) {
                            printf("%lu ", data.dict[i]);
                        }
                    }
                    printf("\n");
                } else if (strcmp(optarg, "set") == 0 && optind < argc) {
                    if (read_dict(argv[optind], &data) == -1) {
                        cleanup(device_fd, dict_fd);
                        exit(EXIT_FAILURE);
                    }

                    if (ioctl(device_fd, SET_DICTIONARY, &data) == -1) {
                        perror("SET_DICTIONARY");
                        cleanup(device_fd, dict_fd);
                        exit(EXIT_FAILURE);
                    }
                } else {
                    fprintf(stderr, "invalid command\n");
                }
                break;
            }
            default: usage();
        }
    }

    cleanup(device_fd, dict_fd);
    exit(EXIT_SUCCESS);
}

int
read_dict(char* file, led_ioctl_data* data) {
    int fd = -1;
    if ((fd = open(file, O_RDONLY, 0)) == -1) {
        fprintf(stderr, "failed to open dictionary file\n");
        return -1;
    }

    // read file
    char buf[FILE_BUF_LEN] = { '\0' };
    if (read(fd, buf, FILE_BUF_LEN) == -1) {
        fprintf(stderr, "failed to read file\n");
        close(fd);
        return -1;
    }
    close(fd);
    buf[FILE_BUF_LEN - 1] = '\0';

    // parse characters and convert to u_ints
    int dict_index = 0;
    char* token    = strtok(buf, ", ");
    while (token != NULL && dict_index < DICT_SIZE) {
        uint64_t code = strtoull(token, NULL, 10);
        if (code == 0ULL) {
            token = strtok(NULL, ", ");
            continue;
        }
        data->dict[dict_index++] = code;
        token                    = strtok(NULL, ", ");
    }
    return 0;
}
