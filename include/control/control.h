#ifndef CONTROL_H
#define CONTROL_H

#include <zephyr.h>
#include <math.h>

#include "odometry.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define MIN_DUTY -30
#define MAX_DUTY 30
#define QUADRAMP_STEP 6
#define ANGLE_THRESHOLD (REVS_PER_RAD/360*5)

#define CAP(_min, _max, _val) \
    MIN(_max, MAX(_min, _val))

#define QUADRAMP(_ref, _val, _step) \
    CAP(_ref - _step, _ref + _step, _val)

#define FREQ_CONTROL_HZ 100

#define CONTROL_FF_L ((float)(1.0/448.0))
#define CONTROL_P_L ((float)(-0.8/448.0))
#define CONTROL_I_L ((float)(-0.08/448.0))
#define CONTROL_FF_R ((float)(1.0/448.0))
#define CONTROL_P_R ((float)(-0.8/448.0))
#define CONTROL_I_R ((float)(-0.08/448.0))


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
void test_angle();

uint8_t is_angle_ok();

void set_robot_speed(speed_t speed);
void set_angle_dest(float a_rad);

#endif // CONTROL_H

