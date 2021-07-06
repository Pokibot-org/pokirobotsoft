#ifndef PATHFINDING_TYPES_H
#define PATHFINDING_TYPES_H

#include <stdint.h>
#include "robot_utils.h"

typedef struct path_node
{
    struct path_node * parent_node;
    coordinates_t coordinate;
    uint8_t is_used;
    long_distance_t distance_to_start;
}path_node_t;

/**
 * @brief Specify the path search space 
 * example : for a space of 2m*2m with a resolution of 1mm
 * boundaries.x = 2000;
 * boundaries.y = 2000;
 */
typedef struct boundaries
{
    int32_t max_x;
    int32_t max_y;
    int32_t min_x;
    int32_t min_y;
}boundaries_t;

#endif