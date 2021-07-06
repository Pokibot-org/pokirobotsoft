#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>

#include "pathfinding_test.h"
#include "odometry.h"

LOG_MODULE_REGISTER(main);


int main(void) {

    LOG_INF("boot\n");

    pathfinding_test_main();
    //test_encoders();
    //test_speed();
    // test_pos();

    return 0;

}
