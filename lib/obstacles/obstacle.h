#ifndef OBSTACLE_H
#define OBSTACLE_H
#include <stdint.h>
#include "utils.h"

#define OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE 512
#define OBSTACLE_HOLDER_ERROR_NONE 0
#define OBSTACLE_HOLDER_ERROR_TO_FULL 1
#define OBSTACLE_HOLDER_ERROR_INVALID_INDEX 2

#define OBSTACLE_COLLISION_ERROR_NONE 0
#define OBSTACLE_COLLISION_ERROR_UNSUPPORTED 255

typedef struct circle
{
    coordinates_t coordinates;
    uint16_t diameter;    
}circle_t;

typedef struct rectangle
{
    coordinates_t coordinates;
    uint16_t width;    
    uint16_t height;    
}rectangle_t;


typedef enum {
    obstacle_type_none,
    obstacle_type_circle,
    obstacle_type_rectangle
}Obstacle_type_t;

typedef struct obstacle
{
    Obstacle_type_t type :2;
    union
    {
        rectangle_t rectangle;
        circle_t circle;
    }data;
}obstacle_t;


typedef struct obstacle_holder
{
    obstacle_t obstacles[OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE];
    uint16_t most_left_known_free_item;
}obstacle_holder_t;


uint8_t obstacle_are_they_colliding(obstacle_t *a, obstacle_t *b);

uint8_t obstacle_holder_push(obstacle_holder_t *obj, obstacle_t *obstacle);
uint8_t obstacle_holder_delete_index(obstacle_holder_t *obj, uint16_t index);
uint8_t obstacle_holder_delete(obstacle_holder_t *obj, obstacle_t *obstacle);

#ifdef UNIT_TEST
    uint8_t obstacle_holder_compact(obstacle_holder_t *obj);
#endif

#endif