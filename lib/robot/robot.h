#ifndef ROBOT_H
#define ROBOT_H
#include "utils.h"

typedef struct robot
{
    coordinates_t position;
    uint16_t angle_degree_times_hundred;
}robot_t;


#endif