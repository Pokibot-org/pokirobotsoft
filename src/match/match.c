#include "zephyr.h"
#include "match/match.h"
#include "strategy.h"
#include "logging/log.h"
#include "tirette.h"

LOG_MODULE_REGISTER(match);

uint8_t go_to_lighthouse(const goal_t * gl)
{
    return 0;
}

uint8_t do_wait(const goal_t * gl)
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

    tirette_init();

    while (!tirette_is_removed())
    {
        k_sleep(K_MSEC(1));
    }

    STRATEGY_BUILD_INIT(strat)
    STRATEGY_BUILD_ADD(NULL, go_to_lighthouse, NULL, NULL, status_ready)
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
