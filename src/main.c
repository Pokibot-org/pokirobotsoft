#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>

#include "odometry.h"

LOG_MODULE_REGISTER(main);


int main(void) {

    LOG_INF("boot\n");

    //test_encoders();
    //test_speed();
    test_pos();

    return 0;

}
