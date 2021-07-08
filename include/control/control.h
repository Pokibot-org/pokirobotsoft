#ifndef CONTROL_H
#define CONTROL_H

#include <zephyr.h>

#include "odometry.h"

#define FREQ_CONTROL_HZ 100

#define CONTROL_FF_L ((float)(1.0/448.0))
#define CONTROL_P_L ((float)(-0.2/448.0/100.0))
#define CONTROL_I_L 0.0f
#define CONTROL_FF_R ((float)(1.0/448.0))
#define CONTROL_P_R ((float)(-0.2/448.0/100.0))
#define CONTROL_I_R 0.0f


#define CONTROL_FIFO_DEPTH 100

typedef struct control_fifo {
    speed_t set_speed;
    int16_t val_l;
    int16_t val_r;
    int32_t err_fsum_l;
    int32_t err_fsum_r;
    int16_t errs_l[CONTROL_FIFO_DEPTH];
    int16_t errs_r[CONTROL_FIFO_DEPTH];
    uint16_t idx;
} control_fifo_t;

void test_motors_speed();
void test_control();

void set_robot_speed(speed_t speed);

#endif // CONTROL_H

