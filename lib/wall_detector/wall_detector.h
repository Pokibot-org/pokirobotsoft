#ifndef WALL_DETECTOR_H
#define WALL_DETECTOR_H
#include "zephyr.h"
#include "device.h"
#include "drivers/gpio.h"


typedef enum {
    wd_front_l = 0,
    wd_front_r = 1,
    wd_back_l = 2,
    wd_back_r = 3,
}wd_name_t;

uint8_t wall_detector_init();
uint8_t wd_get_collision_bitmask();
uint8_t wd_front_is_touching();
uint8_t wd_back_is_touching();
uint8_t wd_is_activated(wd_name_t name);

void test_wall_detector();

#endif