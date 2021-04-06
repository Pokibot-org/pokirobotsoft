#include <zephyr.h>
#include <logging/log.h>
#include "lidar.h"
#include "relative_obstacle_storing.h"
#include "robot.h"

LOG_MODULE_REGISTER(obstacle_manager);


// PRIVATE VAR
static obstacle_holder_t ob_holder = {0};
K_SEM_DEFINE(obsacle_holder_lock, 1, 1);
// PRIVATE DEF
#define CAMSENSE_CENTER_OFFSET_DEG 16.0f

// FUNC

uint8_t obstacle_manager_get_obstacle_snapshot(obstacle_holder_t *obj){
    k_sem_take(&obsacle_holder_lock, K_FOREVER);
    memcpy(obj, &ob_holder, sizeof(ob_holder));
    k_sem_give(&obsacle_holder_lock);
    return 0;
}

static void obstacle_manager_task()
{
    camsense_x1_init();

    lidar_message_t message;
    int err;
    while (true)
    {
        camsense_x1_read_sensor(&message);
        k_sem_take(&obsacle_holder_lock, K_FOREVER);
        err = relative_obstacle_storing_lidar_points_relative_to_robot(&ob_holder, &message, robot_get_obj(), CAMSENSE_CENTER_OFFSET_DEG);
        k_sem_give(&obsacle_holder_lock);
        if (err)
        {
            LOG_ERR("obstacle_manager_task error when calling obstacle_manager_store_lidar_points %d", err);
        }
    }
}

#if CONFIG_OBSTACLE_MANAGER_THREAD_ENABLED
K_THREAD_DEFINE(obstacle_manager_task_name, CONFIG_OBSTACLE_MANAGER_THREAD_STACK, obstacle_manager_task, NULL, NULL, NULL, CONFIG_OBSTACLE_MANAGER_THREAD_PRIORITY, 0, 0);
#endif
