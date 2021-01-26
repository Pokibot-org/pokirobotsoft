#include <zephyr.h>
#include <logging/log.h>
#include "camsense_x1.h"

LOG_MODULE_REGISTER(obstacle_manager);



static void obstacle_manager_task(){
    camsense_x1_init();

    lidar_message_t message;
    while (true)
    {
        camsense_x1_read_sensor(&message);
    }
    
}

K_THREAD_DEFINE(obstacle_manager_task_name, 1024, obstacle_manager_task, NULL, NULL, NULL,
		1, 0, 0);