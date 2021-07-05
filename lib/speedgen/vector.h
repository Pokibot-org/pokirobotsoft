#ifndef VECTOR_UTILS_H
#define VECTOR_UTILS_H
#include <stdint.h>
#include "common_types.h"

int32_t utils_dot_product(const vector_t *a, const vector_t *b);
uint32_t utils_vector_norm(const vector_t *a);
float utils_get_vector_angle(const vector_t *a, const vector_t *b);
vector_t utils_vector_from_points(const coordinates_t *a, const coordinates_t *b);
vector_t utils_get_unit_vector(const vector_t *a);
coordinates_t utils_coordinates_from_vector(const vector_t *a, const coordinates_t *b);
vector_t utils_form_vector(const vector_t *a, const uint32_t *norm);

#endif