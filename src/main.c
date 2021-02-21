#include <zephyr.h>
#include <logging/log.h>
#include "path_manager.h"

LOG_MODULE_REGISTER(main_task);

uint32_t start;
void tmp_clbk(const path_node_t * node, void * ucfg){
    uint32_t end = k_cycle_get_32();

    LOG_INF("Finding path took %f sec\n", (float)(end-start)/sys_clock_hw_cycles_per_sec());
}


void main(void)
{
    LOG_INF("Main Booted\n");
    start = k_cycle_get_32();
    path_manager_config_t cfg = {
        .found_path_clbk = tmp_clbk,
        .found_updated_path_clbk = NULL,
        .user_config = NULL
    };
    coordinates_t start = {
        .x = 10,
        .y = 10,
    };
    coordinates_t end = {
        .x = 2990,
        .y = 10,
    };

    path_manager_find_path(start, end, cfg);
}