#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <zephyr.h>

#include "as5047p.h"

#define FREQ_ODOMETRY_HZ 1000

#define SIGN_L  -
#define SIGN_R  +
#define REVS_PER_RAD    4669000 // 3649892 //3821877 // 50*2^16 * (491/67) == 2*PI
#define RADS_PER_REV    ((float)(1.0/(double)REVS_PER_RAD)) // 50*2^16 * (491/67) == 2*PI
#define DEGS_PER_REV(a) (int32_t)((float)a*RADS_PER_REV*180.0f/3.1415f)

#define ENCODERS_SPI_DEV        DT_LABEL(DT_ALIAS(spi_as5047p))
#define ENCODERS_CS_GPIO_DEV    DEVICE_DT_GET(DT_NODELABEL(gpioa))
#define ENCODER_LEFT_CS_PIN     15
#define ENCODER_RIGHT_CS_PIN    4

#define SPEED_FIFO_DEPTH    4


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
    int32_t a;
    float a_rad;
} pos_t;

void test_encoders();
void test_speed();
void test_pos();

speed_t robot_get_speed();
speed_t robot_get_speed_latest();
void robot_set_pos(pos_t pos);
void robot_set_angle(float rad);
pos_t robot_get_pos();

#endif // ODOMETRY_H

