#include <zephyr.h>
#include <logging/log.h>
#include <math.h>

#include <as5047p.h>
#include "odometry.h"


LOG_MODULE_REGISTER(odometry);


// ======== DEFINES ============================================================


// ======== PRIVATE VARS =======================================================
static speed_fifo_t robot_sfifo;
static pos_t robot_pos;

// ======== PRIVATE FUNCTIONS ==================================================

static int init_encoders(as5047p_t* enc_l, as5047p_t* enc_r) {
    LOG_DBG("init encoders");
    const struct device* spi = device_get_binding(ENCODERS_SPI_DEV);
    if (!spi) {
        LOG_ERR("failed to get SPI");
        return -1;
    }
    as5047p_init(enc_l, spi, 0, ENCODERS_CS_GPIO_DEV, ENCODER_LEFT_CS_PIN);
    as5047p_init(enc_r, spi, 0, ENCODERS_CS_GPIO_DEV, ENCODER_RIGHT_CS_PIN);
    return 0;
}

static void speed_fifo_update(speed_fifo_t* sfifo, speed_t speed) {
    sfifo->idx = (sfifo->idx + 1) % SPEED_FIFO_DEPTH;
    sfifo->mavg.sl -= sfifo->speeds[sfifo->idx].sl / SPEED_FIFO_DEPTH;
    sfifo->mavg.sr -= sfifo->speeds[sfifo->idx].sr / SPEED_FIFO_DEPTH;
    sfifo->speeds[sfifo->idx] = speed;
    sfifo->mavg.sl += sfifo->speeds[sfifo->idx].sl / SPEED_FIFO_DEPTH;
    sfifo->mavg.sr += sfifo->speeds[sfifo->idx].sr / SPEED_FIFO_DEPTH;
}

static void pos_update(pos_t* pos, speed_t speed) {
    int32_t forward = (speed.sl + speed.sr);
    int32_t rotate  = (speed.sr - speed.sl);
    //LOG_DBG("======= forward: %d  ----  rotate: %d", forward, rotate);
    pos->a += rotate / 2;
    pos->a_rad = ((double)pos->a) * RADS_PER_REV;
    pos->x += (int32_t) (forward * cosf(pos->a_rad));
    pos->y += (int32_t) (forward * sinf(pos->a_rad));
    pos->a += rotate / 2;
}


// ======== PUBLIC FUNCTIONS ===================================================

void test_encoders() {
    LOG_INF("starting encoder test");
    as5047p_t enc_l, enc_r;
    if (init_encoders(&enc_l, &enc_r)) {
        LOG_ERR("failed to init encoders");
    }
    while (1) {
        uint16_t val_l, val_r;
        int ret_l = as5047p_transeive(&enc_l, 0xffff, &val_l);
        int ret_r = as5047p_transeive(&enc_r, 0xffff, &val_r);
        LOG_DBG("left  (ret=%d): par=%x err=%x  val=%d  ----  "
                "right (ret=%d): par=%x err=%x  val=%d",
                ret_l,
                (val_l & (1<<15)) >> 15,
                (val_l & (1<<14)) >> 14, val_l & 0x3fff,
                ret_r,
                (val_r & (1<<15)) >> 15,
                (val_r & (1<<14)) >> 14, val_r & 0x3fff);
        k_sleep(K_MSEC(1000));
    }
}

void test_speed() {
    LOG_INF("starting speed test");
    while (1) {
        speed_t speed = robot_get_speed_latest();
        LOG_DBG("speed l: %d  ----  r: %d", speed.sl, speed.sr);
        k_sleep(K_MSEC(1000));
    }
}
void test_pos() {
    LOG_INF("starting position test");
    while (1) {
        pos_t pos = robot_get_pos();
        LOG_DBG("pos x: %9d  ----  y: %9d  ----  a: %9d (%9d)",
                pos.x, pos.y, pos.a, (int)(cosf(pos.a_rad)*1000.0f));
        k_sleep(K_MSEC(1000));
    }
}

speed_t robot_get_speed() {
    return robot_sfifo.mavg;
}

speed_t robot_get_speed_latest() {
    return robot_sfifo.speeds[robot_sfifo.idx];
}

void robot_set_pos(pos_t pos) {
    robot_pos = pos;
}

void robot_set_angle(float rad)
{
    robot_pos.a_rad = rad;
    robot_pos.a=(int32_t)(rad*REVS_PER_RAD);
}


pos_t robot_get_pos() {
    return robot_pos;
}

// ======== TASK ===============================================================

static void odometry_task() {
    LOG_INF("starting odomerty task");
    as5047p_t enc_l, enc_r;
    uint16_t val_l;
    uint16_t val_r;
    if (init_encoders(&enc_l, &enc_r)) {
        LOG_ERR("failed to init encoders");
    }
    int ret_l = as5047p_transeive(&enc_l, 0xffff, &val_l);
    int ret_r = as5047p_transeive(&enc_r, 0xffff, &val_r);
    val_l = SIGN_L val_l;
    val_r = SIGN_R val_r;
    if (ret_r || ret_l) {
        LOG_ERR("failed to read as5047p: ret_l=%d    ret_r=%d",
                ret_l, ret_r);
    }
    // TODO init pos relative to side
    robot_set_pos((pos_t){0, 0, 0, 0.0f});
    while(1) {
        uint16_t newval_l, newval_r;
        int ret_l = as5047p_transeive(&enc_l, 0xffff, &newval_l);
        int ret_r = as5047p_transeive(&enc_r, 0xffff, &newval_r);
        if (ret_r || ret_l) {
            LOG_ERR("failed to read as5047p: ret_l=%d    ret_r=%d",
                    ret_l, ret_r);
        }
        newval_l = SIGN_L newval_l;
        newval_r = SIGN_R newval_r;
        speed_t speed = {
            .sl = (int16_t)(newval_l - val_l),
            .sr = (int16_t)(newval_r - val_r)
        };
        val_l = newval_l;
        val_r = newval_r;
        speed_fifo_update(&robot_sfifo, speed);
        pos_update(&robot_pos, speed);
        k_sleep(K_USEC(1000000 / FREQ_ODOMETRY_HZ));
    }
}

#if CONFIG_ODOMETRY_THREAD_ENABLED
K_THREAD_DEFINE(
        odometry_task_name,
        CONFIG_ODOMETRY_THREAD_STACK,
        odometry_task,
        NULL,
        NULL,
        NULL,
        CONFIG_ODOMETRY_THREAD_PRIORITY,
        0,
        0);
#else
    const k_tid_t odometry_task_name= {0};
#endif
