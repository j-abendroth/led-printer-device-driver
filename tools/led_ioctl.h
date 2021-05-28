#ifndef GPIO_IOCTL_H
#define GPIO_IOCTL_H
#include <sys/ioctl.h>
#include <inttypes.h>
 
#define DICT_SIZE         128
typedef struct led_ioctl_dict {
    uint64_t dict[DICT_SIZE];
} led_ioctl_data;

#define GET_DICTIONARY    _IOR('L', 1, led_ioctl_data)
#define SET_DICTIONARY    _IOW('L', 2, led_ioctl_data)
#define SET_INPUT_PIN     _IOW('L', 3, int)
#define SET_OUTPUT_PIN    _IOW('L', 4, int)
#define SET_MIN_TIME      _IOW('L', 5, int) 


#endif
