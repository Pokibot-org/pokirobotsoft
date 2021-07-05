#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H
#include <stdint.h>

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

#endif
