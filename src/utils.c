#include <math.h>
#include <stdlib.h>

#include "utils.h"
#ifdef UNIT_TEST
#include "stdlib.h"
#else
#include <zephyr.h>
#include <random/rand32.h>
#include "xoshiro128plusplus.h"
#endif

// TODO
// Remove one liner functions by macros
// Or inline functions when type safety is a must

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

uint32_t utils_distance_aproximated(const coordinates_t *a, const coordinates_t *b)
{
    // https://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
    // https://gamedev.stackexchange.com/questions/69241/how-to-optimize-the-distance-function
    uint32_t dx = abs(a->x - b->x);
    uint32_t dy = abs(a->y - b->y);
    return (dx + dy + utils_max(dx, dy)) >> 1;
}


#ifdef UNIT_TEST
uint32_t utils_get_rand32()
{
    return rand();
}

#else
uint32_t utils_get_rand32()
{
    return xoshiro128plusplus_next();
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

// TODO
// Move into #define MAX(a, b) a < b ? b : a for untyped + inlined version
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
