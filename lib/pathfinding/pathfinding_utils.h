#ifndef PATHFINDING_UTILS_H
#define PATHFINDING_UTILS_H
#include "pathfinding_types.h"

#define SQUARE(a) ((a)*(a))

uint32_t utils_distance(coordinates_t a, coordinates_t b);
uint32_t utils_distance_squared(coordinates_t a, coordinates_t b);
uint32_t utils_distance_summed(coordinates_t a, coordinates_t b);
uint32_t utils_get_rand32();
// void utils_normalize_coordinates(coordinates_t *crd);
uint32_t utils_max(uint32_t a, uint32_t b);

#endif