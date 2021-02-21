#include <zephyr.h>
#include "path_manager.h"
#include "pathfinding.h"
#include "string.h"
#include "logging/log.h"

LOG_MODULE_REGISTER(path_manager);

// DEFINES
#define PATH_MANAGER_STACK_SIZE 2048
#define PATH_MANAGER_PRIORITY 2

typedef struct path_manager_object
{
    coordinates_t start;
    coordinates_t end;
    path_manager_config_t conf;
    pathfinding_object_t * pathfinding_obj;
    obstacle_holder_t * obstacle_hold;
}path_manager_object_t;


// PRIVATE VAR
K_THREAD_STACK_DEFINE(path_manager_stack_area, PATH_MANAGER_STACK_SIZE);
struct k_thread path_manager_thread_data;
k_tid_t path_manager_tid = {0};
static pathfinding_object_t path_obj = {0};
static obstacle_holder_t ob_hold = {0};
static path_manager_object_t pm_obj = {0};


// PRIVATE FUN
static void path_manager_task(void * p0, void *p1, void *p2){

    path_manager_object_t * pm_obj = (path_manager_object_t *)p0;
    path_node_t *pn_end;
    
    pathfinding_find_path(pm_obj->pathfinding_obj, pm_obj->obstacle_hold, &pm_obj->start, &pm_obj->end, &pn_end);
    
    if (pm_obj->conf.found_path_clbk != NULL) {
        //LOG_INF("Calling found_path_clbk %d", pm_obj->conf.found_path_clbk);
        pm_obj->conf.found_path_clbk(pn_end, &pm_obj->conf.user_config);
    }else{
        LOG_ERR("found_path_clbk is null");
    }
    if (pm_obj->conf.found_updated_path_clbk != NULL){
        // Do obstacle check
        LOG_INF("found_updated_path_clbk not null");
    }
}


// PUBLIC FUN

uint8_t path_manager_find_path(coordinates_t start, coordinates_t end, path_manager_config_t config){
    
    if (config.found_path_clbk == NULL)
    {
        return -2;
    }

    // if(path_manager_tid != NULL){
    //     k_thread_abort(path_manager_tid);
    // }
    pm_obj.start = start;
    pm_obj.end = end;
    pm_obj.conf = config;

    pm_obj.pathfinding_obj = &path_obj;
    pm_obj.obstacle_hold = &ob_hold; // TODO: get new snapshot
    memset(&pm_obj.pathfinding_obj, 0, sizeof(pm_obj.pathfinding_obj));
    memset(&pm_obj.obstacle_hold, 0, sizeof(pm_obj.obstacle_hold)); // FIXME: Get snapshot obstacle manager


    pathfinding_configuration_t pathfinding_config;
    pathfinding_config.field_boundaries.max_x = 3000; // 3m
    pathfinding_config.field_boundaries.max_y = 2000; // 2m
    pathfinding_config.delta_distance = 100; // jump of Xcm
    pathfinding_config.distance_to_destination = 60; // stop when less than 6 cm close tho goal
    pathfinding_config.radius_of_security = 300; // 300 mm
    pathfinding_object_configure(pm_obj.pathfinding_obj, &pathfinding_config);

    path_manager_tid = k_thread_create(&path_manager_thread_data, path_manager_stack_area,
                                    K_THREAD_STACK_SIZEOF(path_manager_stack_area),
                                    path_manager_task,
                                    &pm_obj, NULL, NULL,
                                    PATH_MANAGER_PRIORITY, 0, K_NO_WAIT);
    if(path_manager_tid == NULL){
        return -1;
    }
    return 0;
}