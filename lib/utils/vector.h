#ifndef VECTOR_UTILS_H
#define VECTOR_UTILS_H
#include "utils.h"

#define DOT(a,b) ((a.x)*(b.x) + (a.y)*(b.y))
#define VECTOR_NORM(a) (sqrtf(SQUARE((int32_t)a.x) + SQUARE((int32_t)a.y)))

float vector_get_vector_angle(const vector_t *a, const vector_t *b);
vector_t vector_vector_from_points(const coordinates_t *a, const coordinates_t *b);
vector_t vector_get_unit_vector(const vector_t *a);
coordinates_t vector_coordinates_from_vector(const vector_t *a, const coordinates_t *b);
vector_t vector_form_vector(const vector_t *a, const uint32_t *norm);
vector_t vector_get_vector_from_angle(const float *a);

#endif