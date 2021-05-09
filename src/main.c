#include <zephyr.h>
#include <logging/log.h>
#include "path_manager.h"
#include "speedgen.h"

LOG_MODULE_REGISTER(main_task);

uint32_t time_start;
void tmp_clbk(const path_node_t * node, void * ucfg){
    uint32_t diff =  k_uptime_get_32() - time_start;
    LOG_INF("Finding path took %u msec", diff);
}

int main(void)
{

    LOG_INF("Main Booted\n");
 
    LOG_INF("Launching pathfinding in 1s\n");
    k_sleep(K_SECONDS(1));
    time_start = k_uptime_get_32();

    path_manager_config_t cfg = {
        .found_path_clbk = tmp_clbk,
        .found_updated_path_clbk = NULL,
        .user_config = NULL
    };
    coordinates_t path_start = {
        .x = 10,
        .y = 10,
    };
    coordinates_t path_end = {
        .x = 2990,
        .y = 10,
    };

    path_manager_find_path(path_start, path_end, cfg);
    while (1)
    {
        k_sleep(K_MSEC(1000));
    }

}
