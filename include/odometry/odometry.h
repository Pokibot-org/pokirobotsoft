#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <zephyr.h>

#include "as5047p.h"

#define FREQ_ODOMETRY_HZ 1000

#define SIGN_L  -
#define SIGN_R  +
#define RADS_PER_REV    (1.0/3821877.0) // 50*2^16 * (491/67) == 2*PI

#define ENCODERS_SPI_DEV        DT_LABEL(DT_ALIAS(spi_as5047p))
#define ENCODERS_CS_GPIO_DEV    DEVICE_DT_GET(DT_NODELABEL(gpioa))
#define ENCODER_LEFT_CS_PIN     4
#define ENCODER_RIGHT_CS_PIN    15

#define SPEED_FIFO_DEPTH    3


// TODO uint32_t ?
typedef struct speed {
    int16_t sl;
    int16_t sr;
} speed_t;

typedef struct speed_fifo {
    speed_t mavg;
    speed_t speeds[SPEED_FIFO_DEPTH];
    uint8_t idx;
} speed_fifo_t;

typedef struct pos {
    uint32_t x;
    uint32_t y;
    uint32_t a;
    float a_rad;
} pos_t;

void test_encoders();
void test_speed();
void test_pos();

speed_t get_speed();
speed_t get_speed_latest();
void set_pos(pos_t pos);
pos_t get_pos();

#endif // ODOMETRY_H

