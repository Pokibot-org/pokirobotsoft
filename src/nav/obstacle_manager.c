#include <zephyr.h>
#include <logging/log.h>
#include "lidar.h"
#include "relative_obstacle_storing.h"
#include "robot.h"

LOG_MODULE_REGISTER(obstacle_manager);

K_MSGQ_DEFINE(obstacle_manager_msgq, sizeof(obstacle_manager_message_t), 10, 1);

// PRIVATE VAR
static obstacle_holder_t ob_holder = {0};
K_SEM_DEFINE(obsacle_holder_lock, 1, 1);
// PRIVATE DEF
#define CAMSENSE_CENTER_OFFSET_DEG 16.0f

// FUNC

void obstacle_manager_send_message(const obstacle_manager_message_t *msg)
{
    while (k_msgq_put(&obstacle_manager_msgq, msg, K_NO_WAIT))
    {
        LOG_ERR("Queue full, purging");
        k_msgq_purge(&obstacle_manager_msgq);
    }
}

uint8_t obstacle_manager_get_obstacle_snapshot(obstacle_holder_t *obj)
{
    k_sem_take(&obsacle_holder_lock, K_FOREVER);
    memcpy(obj, &ob_holder, sizeof(ob_holder));
    k_sem_give(&obsacle_holder_lock);
    return 0;
}

void on_rotation_lidar_callback()
{
    obstacle_manager_message_t msg = {
        .type = obstacle_manager_message_type_event,
        .event = obstacle_manager_message_event_lidar_did_rotation};
    obstacle_manager_send_message(&msg);
}

static void obstacle_manager_task()
{
    camsense_x1_init();
    camsense_x1_add_on_rotation_clbk(on_rotation_lidar_callback);
    int err;
    obstacle_manager_message_t msg;
    while (true)
    {
        k_msgq_get(&obstacle_manager_msgq, &msg, K_FOREVER);
        LOG_DBG("Recived message");
        if (msg.type ==  obstacle_manager_message_type_event)
        {
            LOG_DBG("Recived event");
            if (msg.event == obstacle_manager_message_event_lidar_did_rotation)
            {
                LOG_DBG("Recived lidar_did_rotation event");
                lidar_message_t lidar_message;
                k_sem_take(&obsacle_holder_lock, K_FOREVER);
                while (!camsense_x1_read_sensor_non_blocking(&lidar_message))
                {
                    err = relative_obstacle_storing_lidar_points_relative_to_robot(&ob_holder, &lidar_message, robot_get_obj(), CAMSENSE_CENTER_OFFSET_DEG);
                    if (err)
                    {
                        LOG_ERR("obstacle_manager_task error when calling obstacle_manager_store_lidar_points %d", err);
                    }
                }
                k_sem_give(&obsacle_holder_lock);
            }
        }
    }
}

#if CONFIG_OBSTACLE_MANAGER_THREAD_ENABLED
K_THREAD_DEFINE(obstacle_manager_task_name, CONFIG_OBSTACLE_MANAGER_THREAD_STACK, obstacle_manager_task, NULL, NULL, NULL, CONFIG_OBSTACLE_MANAGER_THREAD_PRIORITY, 0, 0);
#endif
