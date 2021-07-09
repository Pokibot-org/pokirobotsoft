#include <zephyr.h>
#include <kernel.h>
#include <logging/log.h>
#include <math.h>
#include <stdlib.h>

#include "match/match.h"
#include "strategy.h"
#include "tirette.h"
#include "flag/flag.h"
#include "display.h"
#include "common_types.h"
#include "path_manager.h"
#include "obstacle_manager.h"

#include "control.h"
#include "wall_detector.h"
#include "servos.h"

LOG_MODULE_REGISTER(match);

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#define M_PI_2 1.57079632679489661923f
#define M_PI_4 0.78539816339744830962f
#endif

#define TIME_BEFORE_FLAG_RAISE_S 96

static coordinates_t path_storage[1024];
static uint8_t path_is_found = {0};
path_node_t end_node = {0};



void all_servos_up()
{
    int degree = 180/2;
    servos_set(servo_front_l, degree);
    servos_set(servo_front_r, degree);
    servos_set(servo_back_l, degree);
    servos_set(servo_back_r, degree);
}

void all_servos_closed()
{
    int degree = 140;
    servos_set(servo_front_l, degree);
    servos_set(servo_front_r, 180-degree);
    servos_set(servo_back_l, 180-degree);
    servos_set(servo_back_r, degree);
}


static void flag_work_handler(struct k_work *work)
{
    LOG_INF("Raising flag");
    raise_flag();
}

K_DELAYED_WORK_DEFINE(flag_work, flag_work_handler);

void move(int32_t dist_mm)
{
    int8_t sign = dist_mm >= 0 ? 1: -1;
    int16_t motor_speed = sign * 4000;
    int64_t time_start = k_uptime_get();
    int32_t time_to_wait = abs(dist_mm) * 2.5;
    while (k_uptime_get() - time_start < time_to_wait)
    {
        set_robot_speed((speed_t){.sl=motor_speed, .sr=motor_speed});
        k_sleep(K_MSEC(10));
        if (obstacle_manager_is_there_an_obstacle())
        {
            set_robot_speed((speed_t){.sl=0, .sr=0});
        }
    }
    set_robot_speed((speed_t){.sl=0, .sr=0});
}

uint8_t recalibration_back(int32_t timeout_ms)
{
    int16_t recal_speed= -4000;
    int64_t init_time = k_uptime_get();
    while (k_uptime_get() - init_time < timeout_ms)
    {
        if (wd_back_is_touching())
        {
            set_robot_speed((speed_t){.sl=0, .sr=0});
            k_sleep(K_MSEC(50));
            LOG_INF("Back recal ok");
            return 0;
        }
        speed_t sp = {0};
        if (!wd_is_activated(wd_back_l)){
            sp.sl = recal_speed;
        }
        if (!wd_is_activated(wd_back_r)){
            sp.sr = recal_speed;
        }
        set_robot_speed(sp);
        k_sleep(K_MSEC(10));
    }

    LOG_WRN("Back recal timeout");

    return 1;
}

uint8_t recalibration_front(int32_t timeout_ms)
{
    int16_t recal_speed= 4000;
    int64_t init_time = k_uptime_get();
    while (k_uptime_get() - init_time < timeout_ms)
    {
        if (wd_front_is_touching())
        {
            set_robot_speed((speed_t){.sl=0, .sr=0});
            k_sleep(K_MSEC(50));
            LOG_INF("Front recal ok");
            return 0;
        }
        speed_t sp = {0};
        if (!wd_is_activated(wd_front_l)){
            sp.sl = recal_speed;
        }
        if (!wd_is_activated(wd_front_r)){
            sp.sr = recal_speed;
        }
        set_robot_speed(sp);
        k_sleep(K_MSEC(10));
    }

    LOG_WRN("Front recal timeout");

    return 1;
}

uint8_t do_match(const goal_t * gl)
{    
    all_servos_closed();
    recalibration_back(3000);
    robot_set_angle(0);
    move(150);
    set_angle_dest(M_PI/2);
    while (!is_angle_ok())
    {
        k_sleep(K_MSEC(10));
    }
    recalibration_front(6000);
    robot_set_angle(M_PI/2);
    move(-180);

    // callage gauche

    servos_set(servo_front_l, 35);
    servos_set(servo_front_r, 180-35);
    k_sleep(K_MSEC(500));

    set_angle_dest(M_PI/50);
    while (!is_angle_ok())
    {
        k_sleep(K_MSEC(10));
    }
    move(800); 
    
    all_servos_closed();
    // ok action 1

    set_angle_dest(-4*M_PI/5);
    while (!is_angle_ok())
    {
        k_sleep(K_MSEC(10));
    }

    recalibration_front(6000);
    robot_set_angle(-M_PI);

    move(-210);

    set_angle_dest(-M_PI/2);
    while (!is_angle_ok())
    {
        k_sleep(K_MSEC(10));
    }

    // zone depart go droite 

    set_robot_speed(    (speed_t){.sl=4000, .sr=4100});
    k_sleep(K_MSEC(3000));

    recalibration_front(7000);
    robot_set_angle(-M_PI/2);

    move(-180);

    servos_set(servo_front_l, 35);
    servos_set(servo_front_r, 180-35);

    k_sleep(K_MSEC(500));

    set_angle_dest(0);
    while (!is_angle_ok())
    {
        k_sleep(K_MSEC(10));
    }
    move(800); 

    // ok act 2

    set_angle_dest(4*M_PI/5);
    while (!is_angle_ok())
    {
        k_sleep(K_MSEC(10));
    }

    recalibration_front(6000);
    
    set_robot_speed((speed_t){.sl=0, .sr=0});

    display_init();
    display_send(48);

    return 0;
}

// void pathfindig_callback(const path_node_t *node, void *ucfg)
// {
//     end_node = *node;
//     path_is_found = 1;
// }

// uint8_t go_to_lighthouse(const goal_t *gl)
// {
//     coordinates_t *found_path;
//     pos_t robot_pos = robot_get_pos();
//     coordinates_t dest_pos = {.x = 25, .y = 25};

//     path_manager_config_t cfg = {
//         .found_path_clbk = pathfindig_callback,
//         .found_updated_path_clbk = NULL,
//         .user_config = NULL,
//     };

//     path_manager_find_path((coordinates_t){.x = robot_pos.x, .y = robot_pos.y}, dest_pos, cfg);
//     while (!path_is_found)
//     {
//         k_sleep(K_MSEC(50));
//     }
//     path_is_found = 0;

//     uint16_t nb_crd_in_path = path_manager_retrieve_path(path_storage,
//                                                          ARRAY_SIZE(path_storage),
//                                                          &found_path,
//                                                          &end_node);

//     for (size_t i = 0; i < nb_crd_in_path; i++)
//     {
//         LOG_INF("Node x:%d | y:%d", found_path[i].x, found_path[i].y);
//     }
    

//     // do something with path;
//     return 0;
// }

uint8_t do_wait(const goal_t *gl)
{
    while (1)
    {
        k_sleep(K_SECONDS(1));
    }
    return 0;
}

static void match_task()
{
    LOG_INF("Init match task");
    set_robot_speed((speed_t){0, 0});
    tirette_init();
    flag_init();
    display_init();
    servos_init();
    wall_detector_init();

    display_send(33);

    while (!tirette_is_removed())
    {
        k_sleep(K_MSEC(10));
    }
    k_delayed_work_submit(&flag_work, K_SECONDS(TIME_BEFORE_FLAG_RAISE_S));

    k_sleep(K_MSEC(500));
    set_angle_dest(0);

    STRATEGY_BUILD_INIT(strat)
    STRATEGY_BUILD_ADD(NULL, do_match, NULL, status_ready)
    STRATEGY_BUILD_ADD(NULL, do_wait, NULL, status_always_ready)
    STRATEGY_BUILD_END(strat);
    strategy_run(&strat);
}

#if CONFIG_MATCH_THREAD_ENABLED
K_THREAD_DEFINE(match_task_name,
                CONFIG_MATCH_THREAD_STACK,
                match_task,
                NULL, NULL, NULL,
                CONFIG_MATCH_THREAD_PRIORITY, 0, 0);
#endif
