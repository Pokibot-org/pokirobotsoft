#ifndef PATHFINDING_UTILS_H
#define PATHFINDING_UTILS_H
#include <stdint.h>

#define SQUARE(a) ((a) * (a))
#define ABS(a) ((a) < 0 ? -(a) : (a))

typedef int16_t point_t;
typedef uint16_t distance_t;
typedef uint32_t long_distance_t;
typedef struct coordinates
{
    point_t x;
    point_t y;
} coordinates_t;

typedef struct vector
{
    int32_t x;
    int32_t y;
} vector_t;

uint32_t utils_distance(const coordinates_t *a, const coordinates_t *b);
uint32_t utils_distance_squared(const coordinates_t *a, const coordinates_t *b);
uint32_t utils_distance_summed(const coordinates_t *a, const coordinates_t *b);
uint32_t utils_get_rand32();
// void utils_normalize_coordinates(const coordinates_t *crd);
uint32_t utils_max(uint32_t a, uint32_t b);

#endif