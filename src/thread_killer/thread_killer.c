#include <zephyr.h>
#include "thread_killer/thread_killer.h"
#include "logging/log.h"
#include "control/control.h"

LOG_MODULE_REGISTER(thread_killer);

extern const k_tid_t odometry_task_name;
extern const k_tid_t speed_control_task_name;


static const k_tid_t * threads_to_kill[] = {
        &odometry_task_name,
        &speed_control_task_name,
    };

#define NUMBER_OF_THREADS_TO_KILL   ARRAY_SIZE(threads_to_kill)

static void thread_killer_task()
{
    LOG_INF("Init thread killer");
    k_sleep(K_SECONDS(95));
    LOG_INF("Killing necessary threads, end of match");
    for (size_t i = 0; i < NUMBER_OF_THREADS_TO_KILL; i++)
    {
        if (*threads_to_kill[i] == NULL)
        {
            continue;
        }
        k_thread_abort(*threads_to_kill[i]);
    }
    LOG_INF("Killing necessary threads done");
}

#if CONFIG_THREAD_KILLER_THREAD_ENABLED
K_THREAD_DEFINE(thread_killer_task_name,
                CONFIG_THREAD_KILLER_THREAD_STACK,
                thread_killer_task,
                NULL, NULL, NULL,
                CONFIG_THREAD_KILLER_THREAD_PRIORITY, 0, 0);
#endif
