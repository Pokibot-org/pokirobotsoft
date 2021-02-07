#ifndef PATHFINDING_TYPES_H
#define PATHFINDING_TYPES_H

#include <stdint.h>
#include "utils.h"

typedef struct path_node
{
    struct path_node * parent_node;
    struct path_node * son_node;
    coordinates_t coordinate;
    uint8_t is_used;
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
    // TODO: add min boundaries
}boundaries_t;

#endif