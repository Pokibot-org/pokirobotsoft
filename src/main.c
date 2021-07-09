#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>

#include "pathfinding_test.h"
#include "odometry.h"
#include "servos.h"
#include "tirette.h"
#include "control.h"
#include "motors.h"
#include "wall_detector.h"

LOG_MODULE_REGISTER(main);

int main(void) {

    LOG_INF("boot\n");

    //test_wall_detector();
    //test_tirette();
    //test_servo();
    //pathfinding_test_main();
    //test_encoders();
    //test_speed();
    //test_pos();
    //test_motors_speed();
    //test_control();
    // test_angle();
    while (1)
    {
        k_sleep(K_MSEC(1000));
    }
    return 0;

}
