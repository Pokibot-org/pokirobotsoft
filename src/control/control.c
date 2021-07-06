#include <zephyr.h>
#include <logging/log.h>
#include <math.h>

#include "control.h"
#include "motors.h"
#include "odometry.h"


LOG_MODULE_REGISTER(control);


// ======== DEFINES ============================================================


// ======== PRIVATE VARS =======================================================
static control_fifo_t robot_ctrl_fifo;


// ======== PRIVATE FUNCTIONS ==================================================

static int init_motors() {
    motors_init();
}

static void cmp_ctrl_step(control_fifo_t* ctrl_fifo, int16_t sv_l, int16_t sv_r) {
    int16_t err_l = ctrl_fifo->val_l - sv_l;
    int16_t err_r = ctrl_fifo->val_r - sv_r;

    ctrl_fifo->idx = (ctrl_fifo->idx + 1) % CONTROL_FIFO_DEPTH;
    ctrl_fifo->err_fsum_l -= ctrl_fifo->errs_l[ctrl_fifo->idx];
    ctrl_fifo->err_fsum_r -= ctrl_fifo->errs_r[ctrl_fifo->idx];
    ctrl_fifo->errs_l[ctrl_fifo->idx] = err_l;
    ctrl_fifo->errs_r[ctrl_fifo->idx] = err_r;
    ctrl_fifo->err_fsum_l += err_l;
    ctrl_fifo->err_fsum_r += err_r;

    ctrl_fifo->val_l =  CONTROL_P_L * err_l
                        + CONTROL_I_L * ctrl_fifo->err_fsum_l;
    ctrl_fifo->val_r =  CONTROL_P_R * err_r
                        + CONTROL_I_R * ctrl_fifo->err_fsum_r;
}


// ======== PUBLIC FUNCTIONS ===================================================

void test_motors_speed_step(int16_t sl, int16_t sr, uint16_t duration) {
    for (int i = 0; i < 3; i++) {
        motor_set(MOTOR_L, sl);
        motor_set(MOTOR_R, sr);
        speed_t speed = get_speed();
        LOG_DBG("left duty: %d  speed: %d  ----  right duty: %d  speed: %d",
                sl, speed.sl, sr, speed.sr);
        k_sleep(K_MSEC(duration));
    }
}

void test_motors_speed() {
    motors_init();
    while(1) {
        test_motors_speed_step(0, 0, 1000);
        test_motors_speed_step(10, 10, 1000);
        test_motors_speed_step(25, 25, 1000);
        test_motors_speed_step(50, 50, 1000);
        test_motors_speed_step(0, 0, 1000);
        test_motors_speed_step(-10, -10, 1000);
        test_motors_speed_step(-20, 0, 1000);
        test_motors_speed_step(-20, 20, 1000);
    }
}

// ======== TASK ===============================================================

static void speed_control_task() {
    LOG_INF("starting speed_control task");
    init_motors();
    while(1) {
        // TODO
        // - get desired speed from somewhere
        // - stop motors if there is an obstacle
        cmp_ctrl_step(&robot_ctrl_fifo, 0, 0);
        motor_set(MOTOR_L, robot_ctrl_fifo.val_l);
        motor_set(MOTOR_R, robot_ctrl_fifo.val_r);
        k_sleep(K_USEC(1000000 / FREQ_CONTROL_HZ));
    }
}

#if CONFIG_ODOMETRY_THREAD_ENABLED
K_THREAD_DEFINE(
        speed_control_task_name,
        CONFIG_SPEED_CONTROL_THREAD_STACK,
        speed_control_task,
        NULL,
        NULL,
        NULL,
        CONFIG_SPEED_CONTROL_THREAD_PRIORITY,
        0,
        0);
#endif
