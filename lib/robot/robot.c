#include "robot.h"

static robot_t robot_obj = {
    .radius_mm = 190,
    .position = {500,500},
    .angle_rad = 0,
};

robot_t *robot_get_obj(){
    return &robot_obj;
}

