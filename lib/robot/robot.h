#ifndef ROBOT_H
#define ROBOT_H
#include "utils.h"

typedef struct robot
{
    coordinates_t position;
    // angle_rad is in range <-PI,PI>
    float angle_rad; 
}robot_t;

robot_t *robot_get_obj();

#endif