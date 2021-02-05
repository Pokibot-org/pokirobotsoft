#ifndef PATHFINDING_TYPES_H
#define PATHFINDING_TYPES_H

#include <stdint.h>

typedef uint16_t point_t;

typedef struct coordinates
{
    point_t x;
    point_t y;
}coordinates_t;

typedef struct vector
{
    int32_t x;
    int32_t y;
}vector_t;


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
    uint32_t x;
    uint32_t y;
}boundaries_t;

#endif