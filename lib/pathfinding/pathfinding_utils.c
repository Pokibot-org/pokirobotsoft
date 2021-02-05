#include <math.h>
#include "pathfinding_utils.h"

uint32_t distance(coordinates_t a, coordinates_t b)
{
    return sqrtf(SQUARE((uint32_t)a.x - b.x) + SQUARE((uint32_t)a.y - b.y));
}

uint32_t distance_squared(coordinates_t a, coordinates_t b)
{
    return SQUARE((uint32_t)a.x - b.x) + SQUARE((uint32_t)a.y - b.y);
}

uint32_t distance_summed(coordinates_t a, coordinates_t b)
{
    return ((uint32_t)a.x - b.x) + ((uint32_t)a.y - b.y);
}