#ifndef OBSTACLE_MANAGER_H
#define OBSTACLE_MANAGER_H
#include "obstacle.h"

typedef enum obstacle_manager_message_event
{
    obstacle_manager_message_event_lidar_did_rotation
}obstacle_manager_message_event_t;

typedef enum obstacle_manager_message_type
{
    obstacle_manager_message_type_event
}obstacle_manager_message_type_t;

typedef struct obstacle_manager_message
{
    obstacle_manager_message_type_t type;
    union {
        obstacle_manager_message_event_t event;
    };
}obstacle_manager_message_t;

uint8_t obstacle_manager_get_obstacle_snapshot(obstacle_holder_t *obj);
void obstacle_manager_send_message(const obstacle_manager_message_t *msg);

#endif