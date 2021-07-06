#include <math.h>
#include <stdlib.h>
#include "vector.h"
#include "robot_utils.h"

#ifndef UNIT_TEST
#include <zephyr.h>
#endif

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
vector_t utils_vector_from_points(const coordinates_t *a, const coordinates_t *b)
{
    vector_t vect;
    vect.x = b->x - a->x;
    vect.y = b->y - a->y;

    return vect;
};

//Function calculating the unit vector of a vector
vector_t utils_get_unit_vector(const vector_t *a)
{
    vector_t unit_vector;
    uint32_t norm = utils_vector_norm(a);
    unit_vector.x = a->x/norm;
    unit_vector.y = a->y/norm;
    
    return unit_vector;
};

//Function returning the global coordinates of the point pointed by the vector
coordinates_t utils_coordinates_from_vector(const vector_t *a, const coordinates_t *b)
{
    coordinates_t point;

    point.x = b->x + a->x;
    point.y = b->y + a->y;

    return point;
};

//Function creating from a norm and a unit vector
vector_t utils_form_vector(const vector_t *a, const uint32_t *norm)
{
    vector_t vector;

    vector.x = (int32_t)(*norm) * a->x;
    vector.y = (int32_t)(*norm) * a->y;

    return vector;
};