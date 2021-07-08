#include <zephyr.h>
#include <kernel.h>
#include <logging/log.h>
#include "match/match.h"
#include "strategy.h"
#include "tirette.h"
#include "flag/flag.h"
#include "common_types.h"
#include "path_manager.h"

#include "control.h"

LOG_MODULE_REGISTER(match);

#define TIME_BEFORE_FLAG_RAISE_S 96

static coordinates_t path_storage[1024];
static uint8_t path_is_found = {0};
path_node_t end_node = {0};

static void flag_work_handler(struct k_work *work)
{
    LOG_INF("Raising flag");
    raise_flag();
}

K_DELAYED_WORK_DEFINE(flag_work, flag_work_handler);

uint8_t go_forward(const goal_t *gl)
{
    set_robot_speed((speed_t){.sl = 4000, .sr = 4000});
    k_sleep(K_MSEC(3000));
    set_robot_speed((speed_t){.sl = 0, .sr = 0});
    k_sleep(K_MSEC(3000));
    set_robot_speed((speed_t){.sl = 4000, .sr = -4000});
    k_sleep(K_MSEC(3000));
    set_robot_speed((speed_t){.sl = 4000, .sr = 4000});
    k_sleep(K_MSEC(3000));
    set_robot_speed((speed_t){.sl = 0, .sr = 0});
    return 0;
}

void pathfindig_callback(const path_node_t *node, void *ucfg)
{
    end_node = *node;
    path_is_found = 1;
}

uint8_t go_to_lighthouse(const goal_t *gl)
{
    coordinates_t *found_path;
    pos_t robot_pos = robot_get_pos();
    coordinates_t dest_pos = {.x = 25, .y = 25};

    path_manager_config_t cfg = {
        .found_path_clbk = pathfindig_callback,
        .found_updated_path_clbk = NULL,
        .user_config = NULL,
    };

    path_manager_find_path((coordinates_t){.x = robot_pos.x, .y = robot_pos.y}, dest_pos, cfg);
    while (!path_is_found)
    {
        k_sleep(K_MSEC(50));
    }
    path_is_found = 0;

    uint16_t nb_crd_in_path = path_manager_retrieve_path(path_storage,
                                                         ARRAY_SIZE(path_storage),
                                                         &found_path,
                                                         &end_node);

    for (size_t i = 0; i < nb_crd_in_path; i++)
    {
        LOG_INF("Node x:%d | y:%d", found_path[i].x, found_path[i].y);
    }
    

    // do something with path;
}

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

    while (!tirette_is_removed())
    {
        k_sleep(K_MSEC(1));
    }
    k_sleep(K_MSEC(1000));
    k_delayed_work_submit(&flag_work, K_SECONDS(TIME_BEFORE_FLAG_RAISE_S));

    STRATEGY_BUILD_INIT(strat)
    STRATEGY_BUILD_ADD(NULL, go_forward, NULL, NULL, status_ready)
    STRATEGY_BUILD_ADD(NULL, do_wait, NULL, NULL, status_always_ready)
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
