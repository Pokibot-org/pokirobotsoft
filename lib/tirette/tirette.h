#ifndef TIRETTE_H
#define TIRETTE_H
#include "zephyr.h"
#include "device.h"
#include "drivers/gpio.h"

uint8_t tirette_init();
uint8_t tirette_is_removed();

void test_tirette();

#endif