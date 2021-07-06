#ifndef ROBOT_H
#define ROBOT_H
#include "robot_utils.h"

typedef struct robot
{
    coordinates_t position;
    // angle_rad is in range <-PI,PI>
    float angle_rad; 
    distance_t radius_mm;
}robot_t;

robot_t *robot_get_obj();

#endif
