#ifndef PATHFINDING_UTILS_H
#define PATHFINDING_UTILS_H
#include <stdint.h>
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

int32_t utils_dot_product(const vector_t *a, const vector_t *b);
uint32_t utils_vector_norm(const vector_t *a);
float utils_get_vector_angle(const vector_t *a, const vector_t *b);
vector_t *utils_vector_from_points(const coordinates_t *a, const coordinates_t *b);
vector_t *utils_get_unit_vector(const vector_t *a);
coordinates_t *utils_coordinates_from_vector(const vector_t *a, const coordinates_t *b);
vector_t *utils_form_vector(const vector_t *a, const uint32_t *norm);

#endif