#include <zephyr.h>
#include <logging/log.h>
#include "pathfinding_test.h"

LOG_MODULE_REGISTER(main_task);

int main(void)
{
    LOG_INF("Main Booted\n");
    pathfinding_test_main();
    return 0;
}
