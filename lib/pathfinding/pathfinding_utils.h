#ifndef PATHFINDING_UTILS_H
#define PATHFINDING_UTILS_H
#include "pathfinding_types.h"

#define SQUARE(a) ((a)*(a))

uint32_t distance(coordinates_t a, coordinates_t b);
uint32_t distance_squared(coordinates_t a, coordinates_t b);
uint32_t distance_summed(coordinates_t a, coordinates_t b);

#endif