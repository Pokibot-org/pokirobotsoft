#include <zephyr.h>
#include <logging/log.h>
#include <math.h>

#include "control.h"
#include "motors.h"
#include "odometry.h"
#include "tirette.h"
#include "obstacle_manager.h"

LOG_MODULE_REGISTER(control);


// ======== DEFINES ============================================================


// ======== PRIVATE VARS =======================================================
static control_fifo_t robot_ctrl_fifo;


// ======== PRIVATE FUNCTIONS ==================================================

static int init_motors() {
    motors_init();
    return 0;
}

static void cmp_ctrl_step(control_fifo_t* ctrl_fifo) {
    speed_t out_speed = robot_get_speed();
    int16_t err_l = out_speed.sl - ctrl_fifo->set_speed.sl;
    int16_t err_r = out_speed.sr - ctrl_fifo->set_speed.sr;

    ctrl_fifo->idx = (ctrl_fifo->idx + 1) % CONTROL_FIFO_DEPTH;
    ctrl_fifo->err_fsum_l -= ctrl_fifo->errs_l[ctrl_fifo->idx];
    ctrl_fifo->err_fsum_r -= ctrl_fifo->errs_r[ctrl_fifo->idx];
    ctrl_fifo->errs_l[ctrl_fifo->idx] = err_l;
    ctrl_fifo->errs_r[ctrl_fifo->idx] = err_r;
    //if (ctrl_fifo->val_l > MIN_DUTY && ctrl_fifo->val_l < MAX_DUTY) {
        ctrl_fifo->err_fsum_l += err_l;
        //ctrl_fifo->err_fsum_l += err_r;
    //}
    //if (ctrl_fifo->val_r > MIN_DUTY && ctrl_fifo->val_r < MAX_DUTY) {
        ctrl_fifo->err_fsum_r += err_r;
        //ctrl_fifo->err_fsum_r += err_r;
    //}

    int16_t new_val_l =
        CONTROL_FF_L * (float)ctrl_fifo->set_speed.sl
            + CONTROL_P_L * (float)err_l
            + CONTROL_I_L * (float)ctrl_fifo->err_fsum_l;
    int16_t new_val_r =
        CONTROL_FF_R * (float)ctrl_fifo->set_speed.sr
            + CONTROL_P_R * (float)err_r
            + CONTROL_I_R * (float)ctrl_fifo->err_fsum_r;
    ctrl_fifo->val_l = new_val_l;
    ctrl_fifo->val_r = new_val_r;
    //ctrl_fifo->val_l = CAP(
    //        MIN_DUTY,
    //        MAX_DUTY,
    //        QUADRAMP(ctrl_fifo->val_l, new_val_l, QUADRAMP_STEP));
    //ctrl_fifo->val_r = CAP(
    //        MIN_DUTY,
    //        MAX_DUTY,
    //        QUADRAMP(ctrl_fifo->val_r, new_val_r, QUADRAMP_STEP));
}


// ======== PUBLIC FUNCTIONS ===================================================

void set_robot_speed(speed_t set_speed) {
    robot_ctrl_fifo.set_speed = set_speed;
}

void test_motors_speed_step(int16_t sl, int16_t sr, uint16_t duration) {
    for (int i = 0; i < 3; i++) {
        motor_set(MOTOR_L, sl);
        motor_set(MOTOR_R, sr);
        speed_t speed = robot_get_speed();
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

void test_control_step(speed_t set_speed, uint16_t duration) {
    set_robot_speed(set_speed);
    for (int i = 0; i < 5; i++) {
        LOG_DBG("left duty: %d  set: %d  err: %d  ----  "
                "right duty: %d  set: %d err: %d",
                robot_ctrl_fifo.val_l,
                robot_ctrl_fifo.set_speed.sl,
                robot_ctrl_fifo.errs_l[robot_ctrl_fifo.idx],
                robot_ctrl_fifo.val_r,
                robot_ctrl_fifo.set_speed.sr,
                robot_ctrl_fifo.errs_r[robot_ctrl_fifo.idx]);
        k_sleep(K_MSEC(duration));
    }
}

void test_control() {
    while(1) {
        for (int i = 0; i<12000; i+=1000) {
            test_control_step((speed_t){.sl=i, .sr=i}, 2000);
        }
        test_control_step((speed_t){.sl=0, .sr=0}, 500);
        test_control_step((speed_t){.sl=-12000, .sr=-12000}, 3000);
        test_control_step((speed_t){.sl=0, .sr=0}, 500);
        test_control_step((speed_t){.sl=-8000, .sr=-8000}, 3000);
        test_control_step((speed_t){.sl=0, .sr=0}, 1000);
        test_control_step((speed_t){.sl=-8000, .sr=0}, 1000);
        test_control_step((speed_t){.sl=0, .sr=0}, 1000);
        test_control_step((speed_t){.sl=-6000, .sr=6000}, 1000);
    }
}

// ======== TASK ===============================================================

static void speed_control_task() {
    LOG_INF("starting speed_control task");
    init_motors();
    tirette_init();

    while (!tirette_is_removed())
    {
        k_sleep(K_MSEC(10));
    }

    while(1) {
        // TODO
        // - get desired speed from somewhere
        // - stop motors if there is an obstacle
        cmp_ctrl_step(&robot_ctrl_fifo);
        if (obstacle_manager_is_there_an_obstacle()) {
            robot_ctrl_fifo.val_l = 0;
            robot_ctrl_fifo.val_r = 0;
        }
        motor_set(MOTOR_L, robot_ctrl_fifo.val_l);
        motor_set(MOTOR_R, robot_ctrl_fifo.val_r);
        k_sleep(K_USEC(1000000 / FREQ_CONTROL_HZ));
    }
}

#if CONFIG_SPEED_CONTROL_THREAD_ENABLED
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
#else
const k_tid_t speed_control_task_name = {0};
#endif

//static void pos_control_task() {
//    LOG_INF("starting pos_control task");
//    init_motors();
//    tirette_init();
//
//    while (!tirette_is_removed())
//    {
//        k_sleep(K_MSEC(10));
//    }
//
//    while(1) {
//        // TODO
//        // - get desired speed from somewhere
//        // - stop motors if there is an obstacle
//        cmp_pos_ctrl_step(&robot_ctrl_fifo);
//        k_sleep(K_USEC(1000000 / FREQ_CONTROL_HZ));
//    }
//}
//
//#if CONFIG_POS_CONTROL_THREAD_ENABLED
//K_THREAD_DEFINE(
//        speed_control_task_name,
//        CONFIG_POS_CONTROL_THREAD_STACK,
//        pos_control_task,
//        NULL,
//        NULL,
//        NULL,
//        CONFIG_POS_CONTROL_THREAD_PRIORITY,
//        0,
//        0);
//#else
//const k_tid_t speed_control_task_name = {0};


