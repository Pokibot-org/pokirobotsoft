#include <robot.h>

static robot_t robot_obj;

robot_t *robot_get_obj(){
    return &robot_obj;
}

