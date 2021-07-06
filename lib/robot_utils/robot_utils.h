#ifndef PATHFINDING_UTILS_H
#define PATHFINDING_UTILS_H

#include "common_types.h"

#define SQUARE(a) ((a) * (a))
#define ABS(a) ((a) < 0 ? -(a) : (a))
//#define MAX(a, b) ((a)>(b) ? (a) : (b))


float utils_distance(const coordinates_t *a, const coordinates_t *b);
float utils_distance_squared(const coordinates_t *a, const coordinates_t *b);
float utils_distance_summed(const coordinates_t *a, const coordinates_t *b);
uint32_t utils_get_rand32();

#endif
