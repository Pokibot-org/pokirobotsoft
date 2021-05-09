#include <math.h>
#include <stdlib.h>

#include "utils.h"
#include "xoshiro128plusplus.h"

#ifndef UNIT_TEST
#include <zephyr.h>
#include <random/rand32.h>
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

uint32_t utils_get_rand32()
{
    return xoshiro128plusplus_next();
}

void utils_init_rand_seed(uint32_t tab[4])
{
    int err = xoshiro128plusplus_init_seed(tab);
    if (err) {
        uint32_t s[4] = {
            0x4ae4098c,
            0xde47cafb,
            0x97cd3540,
            0xde4886e6
        };
        xoshiro128plusplus_init_seed(s);
    }
}

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

//Function calculating the dot Product of two Vectors
int32_t utils_dot_product(const vector_t *a, const vector_t *b)
{
    return a->x*b->x + a->y*b->y;
};

//Function calculating the vector Norm
uint32_t utils_vector_norm(const vector_t *a){
    return sqrtf(SQUARE((int32_t)a->x) + SQUARE((int32_t)a->y));
};

//Function calculating the angle between two Vectors in radians
float utils_get_vector_angle(const vector_t *a, const vector_t *b)
{
    int32_t dot_product = utils_dot_product(a,b);
    uint32_t norm_a = utils_vector_norm(a);
    uint32_t norm_b = utils_vector_norm(b);

    return acos(dot_product/(norm_a*norm_b));
};

//Function calculating a vector from point A to B : with Y reference Up and X reference Right
vector_t *utils_vector_from_points(const coordinates_t *a, const coordinates_t *b)
{
    vector_t *vect;
    vect->x = b->x - a->x;
    vect->y = b->y - a->y;

    return vect;
};

//Function calculating the unit vector of a vector
vector_t *utils_get_unit_vector(const vector_t *a)
{
    vector_t *unit_vector;
    uint32_t norm = utils_vector_norm(a);
    unit_vector->x = a->x/norm;
    unit_vector->y = a->y/norm;
    
    return unit_vector;
};

//Function returning the global coordinates of the point pointed by the vector
coordinates_t *utils_coordinates_from_vector(const vector_t *a, const coordinates_t *b)
{
    coordinates_t *point;

    point->x = b->x + a->x;
    point->y = b->y + a->y;

    return point;
};

//Function creating from a norm and a unit vector
vector_t *utils_form_vector(const vector_t *a, const uint32_t *norm)
{
    vector_t *vector;

    vector->x = (int32_t)norm * a->x;
    vector->y = (int32_t)norm * a->y;

    return vector;
};

