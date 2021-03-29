#include <robot.h>

static robot_t robot_obj = {
    .radius_mm = 150
};

robot_t *robot_get_obj(){
    return &robot_obj;
}

