#ifndef PATHFINDING_H
#define PATHFINDING_H
#include <stdint.h>

#include "pathfinding_types.h"
#include "pathfinding_errors.h"

#define PATHFINDING_MAX_NUM_OF_NODES 2048
#define PATHFINDING_MAX_BOUNDARIES UINT16_MAX

typedef struct pathfinding_configuration {
    boundaries_t field_boundaries;
    distance_t distance_to_destination;
    distance_t delta_distance;
}pathfinding_configuration_t;

typedef struct pathfinding_object
{
    path_node_t nodes[PATHFINDING_MAX_NUM_OF_NODES];
    pathfinding_configuration_t config;
}pathfinding_object_t;


int pathfinding_object_configure(pathfinding_object_t *obj, pathfinding_configuration_t *config);
int pathfinding_find_path(pathfinding_object_t *obj, coordinates_t *start, coordinates_t *end, path_node_t **end_node);
#ifdef UNIT_TEST
    int get_new_valid_coordinates(pathfinding_object_t *obj, coordinates_t *crd_tree_node, coordinates_t *crd_random_node, coordinates_t *crd_new_node);
    void pathfinding_debug_print(pathfinding_object_t *obj);
    void pathfinding_debug_print_found_path(pathfinding_object_t *obj, path_node_t *end_node);
#endif

#endif