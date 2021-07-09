#include <zephyr.h>
#include <logging/log.h>
#include "pathfinding_test.h"
#include "path_manager.h"

LOG_MODULE_REGISTER(pathfinding_test);

static uint32_t time_start;
void tmp_clbk(const path_node_t * node, void * ucfg){
    uint32_t diff =  k_uptime_get_32() - time_start;
    LOG_INF("Finding path took %u msec", diff);
}


void pathfinding_test_main()
{
    LOG_INF("Launching pathfinding in 1s\n");
    k_sleep(K_SECONDS(1));
    time_start = k_uptime_get_32();

    path_manager_config_t cfg = {
        .found_path_clbk = tmp_clbk,
        .found_updated_path_clbk = NULL,
        .user_config = NULL,
    };
    coordinates_t path_start = {
        .x = 300,
        .y = 300,
    };
    coordinates_t path_end = {
        .x = 3000-300,
        .y = 2000-300,
    };

    path_manager_find_path(path_start, path_end, cfg);
    while (1)
    {
        k_sleep(K_MSEC(1000));
    }
}