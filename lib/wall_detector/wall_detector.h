#ifndef WALL_DETECTOR_H
#define WALL_DETECTOR_H
#include "zephyr.h"
#include "device.h"
#include "drivers/gpio.h"

uint8_t wall_detector_init();
uint8_t wd_get_collision_bitmask();

void test_wall_detector();

#endif