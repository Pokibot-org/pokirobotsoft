#ifndef CONTROL_H
#define CONTROL_H

#include <zephyr.h>

#define FREQ_CONTROL_HZ 100

#define CONTROL_P_L 1.0f
#define CONTROL_I_L 0.0f
#define CONTROL_P_R 1.0f
#define CONTROL_I_R 0.0f


#define CONTROL_FIFO_DEPTH 100

typedef struct control_fifo {
    int16_t val_l;
    int16_t val_r;
    int32_t err_fsum_l;
    int32_t err_fsum_r;
    int16_t errs_l[CONTROL_FIFO_DEPTH];
    int16_t errs_r[CONTROL_FIFO_DEPTH];
    uint16_t idx;
} control_fifo_t;

void test_motors_raw();

#endif // CONTROL_H

