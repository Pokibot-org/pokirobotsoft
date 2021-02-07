#include <zephyr.h>
#include <logging/log.h>
#include "camsense_x1.h"
#include "obstacle.h"
#include "relative_obstacle_storing.h"
#include "robot.h"

LOG_MODULE_REGISTER(obstacle_manager);


// PRIVATE VAR
static obstacle_holder_t ob_holder = {0};
K_SEM_DEFINE(obsacle_holder_lock, 0, 1);
// PRIVATE DEF
#define CAMNSENSE_CENTER_OFFSET_DEG 16.0f

// FUNC

uint8_t get_obstacle_snaphot(obstacle_holder_t *obj){
    k_sem_take(&obsacle_holder_lock, K_FOREVER);
    *obj = ob_holder;
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
        err = relative_obstacle_storing_lidar_points_relative_to_robot(&ob_holder, &message, robot_get_obj(), CAMNSENSE_CENTER_OFFSET_DEG);
        k_sem_give(&obsacle_holder_lock);
        if (err)
        {
            LOG_ERR("obstacle_manager_task error when calling obstacle_manager_store_lidar_points %d", err);
        }
    }
}

K_THREAD_DEFINE(obstacle_manager_task_name, 1024, obstacle_manager_task, NULL, NULL, NULL,
                1, 0, 0);