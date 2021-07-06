
#ifndef PATHFINDING_UTILS_H
#define PATHFINDING_UTILS_H

#include "common_types.h"


#define SQUARE(a) ((a) * (a))
#define ABS(a) ((a) < 0 ? -(a) : (a))

uint32_t utils_distance(const coordinates_t *a, const coordinates_t *b);
uint32_t utils_distance_squared(const coordinates_t *a, const coordinates_t *b);
uint32_t utils_distance_summed(const coordinates_t *a, const coordinates_t *b);
uint32_t utils_get_rand32();
void utils_init_rand_seed(uint32_t tab[4]);

// void utils_normalize_coordinates(const coordinates_t *crd);
uint32_t utils_max(uint32_t a, uint32_t b);

#endif

