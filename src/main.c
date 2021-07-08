#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>

#include "pathfinding_test.h"
#include "odometry.h"
#include "servos.h"
#include "tirette.h"
#include "strategy.h"

LOG_MODULE_REGISTER(main);

uint8_t go_to_lighthouse(const goal_t * gl)
{
    return 0;
}

uint8_t do_wait(const goal_t * gl)
{
    while (1)
    {
        k_sleep(K_SECONDS(1));
    }
    return 0;
}

int main(void) {

    LOG_INF("boot\n");

    STRATEGY_BUILD_INIT(strat)
    STRATEGY_BUILD_ADD(NULL, go_to_lighthouse, NULL, NULL, status_ready)
    STRATEGY_BUILD_ADD(NULL, do_wait, NULL, NULL, status_ready)
    STRATEGY_BUILD_END(strat);
    strategy_run(&strat);

    // test_tirette();
    // test_servo();
    // pathfinding_test_main();
    //test_encoders();
    //test_speed();
    //test_pos();
    //test_motors_speed();

    return 0;

}
