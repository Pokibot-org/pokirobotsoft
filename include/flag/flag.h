#ifndef FLAG_H
#define FLAG_H
#include "stdint.h"

#define FLAG_PORT_LABEL "GPIOA"
#define FLAG_PIN 11

uint8_t flag_init();
uint8_t raise_flag();

#endif