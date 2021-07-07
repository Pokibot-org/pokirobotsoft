#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>

#include "pathfinding_test.h"
#include "odometry.h"
#include "servos.h"

LOG_MODULE_REGISTER(main);


int main(void) {

    LOG_INF("boot\n");

    test_servo();

    // pathfinding_test_main();
    //test_encoders();
    //test_speed();
    //test_pos();
    //test_motors_speed();

    return 0;

}
