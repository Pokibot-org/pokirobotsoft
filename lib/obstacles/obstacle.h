#ifndef OBSTACLE_H
#define OBSTACLE_H
#include <stdint.h>
#include "utils.h"

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

uint8_t obstacle_are_they_colliding(obstacle_t *a, obstacle_t *b);

#endif