#include <math.h>
#include <stdlib.h>

#include "vector.h"

#ifndef UNIT_TEST
#include <zephyr.h>
#endif

//Function calculating the angle between two Vectors in radians
float vector_get_vector_angle(const vector_t *a, const vector_t *b)
{
    int32_t dot_product = DOT((*a),(*b));
    uint32_t norm_a = VECTOR_NORM((*a));
    uint32_t norm_b = VECTOR_NORM((*b));

    return acos(dot_product/(norm_a*norm_b));
};

//Function calculating a vector from point A to B : with Y reference Up and X reference Right
vector_t vector_vector_from_points(const coordinates_t *a, const coordinates_t *b)
{
    vector_t vect;
    vect.x = b->x - a->x;
    vect.y = b->y - a->y;

    return vect;
};

//Function calculating the unit vector of a vector
vector_t vector_get_unit_vector(const vector_t *a)
{
    vector_t unit_vector;
    uint32_t norm = VECTOR_NORM((*a));
    unit_vector.x = a->x/norm;
    unit_vector.y = a->y/norm;
    
    return unit_vector;
};

//Function returning the global coordinates of the point pointed by the vector
coordinates_t vector_coordinates_from_vector(const vector_t *a, const coordinates_t *b)
{
    coordinates_t point;

    point.x = b->x + a->x;
    point.y = b->y + a->y;

    return point;
};

//Function creating from a norm and a unit vector
vector_t vector_form_vector(const vector_t *a, const uint32_t *norm)
{
    vector_t vector;

    vector.x = (int32_t)norm * a->x;
    vector.y = (int32_t)norm * a->y;

    return vector;
};

vector_t vector_get_vector_from_angle(const float *a)
{
    vector_t vector;

    vector.x = 1/cosf(*a);
    vector.y = 1/sinf(*a);

    return vector;
};