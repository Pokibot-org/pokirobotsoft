#ifndef PATHFINDING_H
#define PATHFINDING_H
#include <stdint.h>

#include "pathfinding_types.h"
// TODO: define pathfinding erros

#define PATHFINDING_MAX_NUM_OF_NODES 1024
#define PATHFINDING_MAX_BOUNDARIES UINT16_MAX
typedef struct pathfinding_configuration {
    boundaries_t field_boundaries;
}pathfinding_configuration_t;

int pathfinding_configure(pathfinding_configuration_t *config);
int pathfinding_find_path(coordinates_t *start, coordinates_t *end);
void pathfinding_debug_print();

#endif