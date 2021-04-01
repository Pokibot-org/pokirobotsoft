#ifndef PATHFINDING_H
#define PATHFINDING_H
#include <stdint.h>

#include "pathfinding_types.h"
#include "pathfinding_errors.h"
#include "obstacle.h"

#define PATHFINDING_MAX_NUM_OF_NODES 1024
#define PATHFINDING_MAX_BOUNDARIES INT16_MAX

typedef struct pathfinding_configuration {
    boundaries_t field_boundaries;
    distance_t distance_to_destination;
    distance_t delta_distance;
    distance_t radius_of_security; // Must be radius of the robot + some
    distance_t node_remapping_distance;
}pathfinding_configuration_t;

typedef struct pathfinding_object
{
    uint32_t next_free_node_nb;
    path_node_t nodes[PATHFINDING_MAX_NUM_OF_NODES];
    pathfinding_configuration_t config;
}pathfinding_object_t;


int pathfinding_object_configure(pathfinding_object_t *obj, pathfinding_configuration_t *config);
int pathfinding_find_path(pathfinding_object_t *obj, obstacle_holder_t *ob_hold, const coordinates_t *start, const coordinates_t *end, path_node_t **end_node);
int pathfinding_optimize_path(pathfinding_object_t *obj, obstacle_holder_t *ob_hold, uint16_t nb_of_nodes_to_add);
uint16_t pathfinding_get_number_of_used_nodes(pathfinding_object_t *obj);
#ifdef UNIT_TEST
    int get_new_valid_coordinates(pathfinding_object_t *obj, coordinates_t *crd_tree_node, coordinates_t *crd_random_node, coordinates_t *crd_new_node);
    void pathfinding_debug_print(pathfinding_object_t *obj);
    void pathfinding_debug_print_found_path(pathfinding_object_t *obj, path_node_t *end_node);
    void pathfinding_debug_write_found_path(pathfinding_object_t *obj, path_node_t *end_node, char* file_path);
    void pathfinding_debug_write_found_path_list(pathfinding_object_t *obj, path_node_t *end_node, char* file_path);

    path_node_t *get_closest_node(pathfinding_object_t *obj, coordinates_t *crd);
#endif

#endif