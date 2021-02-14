#include <math.h>
#include <stdlib.h>

#include "utils.h"
#ifdef UNIT_TEST
#include "stdlib.h"
#else
#include <zephyr.h>
#include <random/rand32.h>
#endif

/*
if opti needed
double fsqrt (double y) {
    double x, z, tempf;
    unsigned long *tfptr = ((unsigned long *)&tempf) + 1;

	tempf = y;
	*tfptr = (0xbfcdd90a - *tfptr)>>1;
	x =  tempf;
	z =  y*0.5;                        
	x = (1.5*x) - (x*x)*(x*z);         
	x = (1.5*x) – (x*x)*(x*z);
	x = (1.5*x) – (x*x)*(x*z);
	x = (1.5*x) – (x*x)*(x*z);
	x = (1.5*x) – (x*x)*(x*z);
	return x*y;
    }
*/

// TODO: Octile distance h(x) = max( (x1 – x2), (y1 – y2) + (sqrt(2) -1) * min( (x1 – x2), (y1 – y2))

uint32_t utils_distance(const coordinates_t *a, const coordinates_t *b)
{
    // TODO: only safe is output is at least 33 bit
    return sqrtf(SQUARE((int32_t)a->x - b->x) + SQUARE((int32_t)a->y - b->y));
}

uint32_t utils_distance_squared(const coordinates_t *a, const coordinates_t *b)
{
    return SQUARE((int32_t)a->x - b->x) + SQUARE((int32_t)a->y - b->y);
}

uint32_t utils_distance_summed(const coordinates_t *a, const coordinates_t *b)
{
    return abs((int32_t)a->x - b->x) + abs((int32_t)a->y - b->y);
}

#ifdef UNIT_TEST
uint32_t utils_get_rand32()
{
    return rand();
}

point_t utils_get_rand_point()
{
    int pt = rand();
    return *(point_t *)&pt;
}
#else
uint32_t utils_get_rand32()
{
    return sys_rand32_get();
}

point_t utils_get_rand_point()
{
    int pt = sys_rand32_get();
    return *(point_t *)&pt;
}
#endif

/* Normalize with int ... ??? 
void utils_normalize_coordinates(const coordinates_t *crd){
    // TODO: check div per 0 ?
    uint32_t max = max(crd->x, crd->y);
    crd->x = crd->x / max;
    crd->y = crd->y / max;
}
*/

uint32_t utils_max(uint32_t a, uint32_t b)
{
    if (a >= b)
    {
        return a;
    }
    else
    {
        return b;
    }
}