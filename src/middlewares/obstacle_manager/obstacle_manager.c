#include <zephyr.h>
#include <logging/log.h>
#include "camsense_x1.h"
#include "obstacle.h"
#include "relative_obstacle_storing.h"
#include "robot.h"

LOG_MODULE_REGISTER(obstacle_manager);

obstacle_holder_t ob_holder = {0};

#define CAMNSENSE_CENTER_OFFSET_DEG 16.0f

static void obstacle_manager_task()
{
    camsense_x1_init();

    lidar_message_t message;
    int err;
    while (true)
    {
        camsense_x1_read_sensor(&message);
        err = relative_obstacle_storing_lidar_points_relative_to_robot(&ob_holder, &message, robot_get_obj(), CAMNSENSE_CENTER_OFFSET_DEG);
        if (err)
        {
            LOG_ERR("obstacle_manager_task error when calling obstacle_manager_store_lidar_points %d", err);
        }
    }
}

K_THREAD_DEFINE(obstacle_manager_task_name, 1024, obstacle_manager_task, NULL, NULL, NULL,
                1, 0, 0);