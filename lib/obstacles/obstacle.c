#include "obstacle.h"
#include "math.h"
#include "common_types.h"
#ifndef UNIT_TEST
#include <zephyr.h>
#define M_PI 3.14159265358979323846f
#define M_PI_2 1.57079632679489661923f
#define M_PI_4 0.78539816339744830962f
#endif

#define OBSTACLE_COLLISION_NB_MAX_SIDES 8

int16_t obstacle_holder_get_number_of_obstacles(obstacle_holder_t *obj)
{
    uint16_t res = 0;
    for (uint16_t i = 0; i < OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE; i++)
    {
        if (obj->obstacles[i].type != obstacle_type_none)
        {
            res += 1;
        }
    }
    return res;
}

uint8_t obstacle_holder_compact(obstacle_holder_t *obj)
{
    int32_t head = -1;
    for (uint32_t i = 0; i < OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE; i++)
    {
        if (obj->obstacles[i].type != obstacle_type_none)
        {
            if (head >= 0)
            {
                obj->obstacles[head] = obj->obstacles[i];
                obj->obstacles[i].type = obstacle_type_none;
                head++;
            }
        }
        else
        {
            if (head == -1)
            {
                head = i;
            }
        }
    }
    if (head >= 0)
    {
        obj->write_head = head;
    }
    return OBSTACLE_HOLDER_ERROR_NONE;
}

uint8_t obstacle_holder_push(obstacle_holder_t *obj, obstacle_t *obstacle)
{
    if (obj->write_head == OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE)
    {
        return OBSTACLE_HOLDER_ERROR_TO_FULL;
    }
    obj->obstacles[obj->write_head] = *obstacle;
    obj->write_head += 1;
    if (obj->write_head == OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE)
    {
        obstacle_holder_compact(obj);
    }
    return OBSTACLE_HOLDER_ERROR_NONE;
}

uint8_t obstacle_holder_get(obstacle_holder_t *obj, obstacle_t **obstacle)
{
    while (1)
    {
        if (obj->read_head == obj->write_head)
        {
            return OBSTACLE_HOLDER_ERROR_NO_OBSTACLE_FOUND;
        }
        if (obj->obstacles[obj->read_head].type != obstacle_type_none)
        {
            *obstacle = &obj->obstacles[obj->read_head];
            return OBSTACLE_HOLDER_ERROR_NONE;
        }
        obj->read_head += 1;
        if (obj->read_head == OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE)
        {
            obj->read_head = 0;
        }
    }
}

uint8_t obstacle_holder_push_circular_buffer_mode(obstacle_holder_t *obj, obstacle_t *obstacle)
{
    obj->obstacles[obj->write_head] = *obstacle;
    obj->write_head += 1;
    if (obj->write_head == OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE)
    {
        obj->write_head = 0;
    }
    return OBSTACLE_HOLDER_ERROR_NONE;
}

uint8_t obstacle_holder_delete_index(obstacle_holder_t *obj, uint16_t index)
{
    if (index >= OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE)
    {
        return OBSTACLE_HOLDER_ERROR_INVALID_INDEX;
    }
    obj->obstacles[index].type = obstacle_type_none;
    return OBSTACLE_HOLDER_ERROR_NONE;
}

/**
 * @brief Delete the obstacle object from the obstacle_holder object.
 * THE OBSTACLE OBJECT ADRESS MUST BE IN THE obstacle_holder_t OBJECT
 */
uint8_t obstacle_holder_delete(obstacle_holder_t *obj, obstacle_t *obstacle)
{
    uint32_t index = (obstacle - obj->obstacles) / sizeof(obstacle_t);
    return obstacle_holder_delete_index(obj, index);
}

uint8_t are_rectangle_and_circle_colliding(const rectangle_t *rec, const circle_t *cir)
{
    uint32_t circle_distance_x = ABS((int32_t)cir->coordinates.x - (int32_t)rec->coordinates.x);
    uint32_t circle_distance_y = ABS((int32_t)cir->coordinates.y - (int32_t)rec->coordinates.y);

    if (circle_distance_x > (rec->width / 2 + cir->radius))
    {
        return 0;
    }
    if (circle_distance_y > (rec->height / 2 + cir->radius))
    {
        return 0;
    }

    if (circle_distance_x <= (rec->width / 2))
    {
        return 1;
    }
    if (circle_distance_y <= (rec->height / 2))
    {
        return 1;
    }

    uint32_t corner_distance_sq = SQUARE((int32_t)circle_distance_x - rec->width / 2) +
                                  SQUARE((int32_t)circle_distance_y - rec->height / 2);

    return (corner_distance_sq <= SQUARE(cir->radius));
}

uint8_t obstacle_are_they_colliding(const obstacle_t *a, const obstacle_t *b)
{
    if ((a->type == obstacle_type_none) || (b->type == obstacle_type_none))
    {
        return OBSTACLE_COLLISION_ERROR_UNSUPPORTED;
    }
    if (a->type == b->type)
    {
        if (a->type == obstacle_type_circle)
        {
            return utils_distance(&a->data.circle.coordinates, &b->data.circle.coordinates) <= ((a->data.circle.radius + b->data.circle.radius));
        }
        else
        {
            return OBSTACLE_COLLISION_ERROR_UNSUPPORTED; // FIXME: NOT SUPPORTED
        }
    }

    if (a->type == obstacle_type_circle)
    {
        return are_rectangle_and_circle_colliding(&b->data.rectangle, &a->data.circle);
    }
    else
    {
        return are_rectangle_and_circle_colliding(&a->data.rectangle, &b->data.circle);
    }
    return OBSTACLE_COLLISION_ERROR_UNSUPPORTED;
}

#include "stdio.h"

uint8_t check_seg_collision(const coordinates_t *a1, const coordinates_t *a2, const coordinates_t *b1, const coordinates_t *b2, coordinates_t *out)
{
    vector_t vec_a, vec_b;
    vec_a.x = a2->x - a1->x;
    vec_a.y = a2->y - a1->y;
    vec_b.x = b2->x - b1->x;
    vec_b.y = b2->y - b1->y;

    int32_t den = vec_a.x * vec_b.y - vec_a.y * vec_b.x;
    if (!den)
    {
        *out = *a1;
        return 0;
    }
    float coeff_point_sur_a = ((float)(-a1->x * vec_b.y + b1->x * vec_b.y + vec_b.x * a1->y - vec_b.x * b1->y)) / den;
    float coeff_point_sur_b = ((float)(vec_a.x * a1->y - vec_a.x * b1->y - vec_a.y * a1->x + vec_a.y * b1->x)) / den;

    if (coeff_point_sur_a > 0 && coeff_point_sur_a < 1 && coeff_point_sur_b > 0 && coeff_point_sur_b < 1)
    {
        out->x = a1->x + coeff_point_sur_a * vec_a.x;
        out->y = a1->y + coeff_point_sur_a * vec_a.y;
        return 1;
    }
    return 0;
}

uint8_t obstacle_get_point_of_collision_with_segment(const coordinates_t *start_point, const coordinates_t *end_point, const obstacle_t *obstacle, const uint16_t *seg_radius, coordinates_t *out_crd)
{
    coordinates_t points[OBSTACLE_COLLISION_NB_MAX_SIDES];
    uint8_t sides_to_check = 0;

    if (obstacle->type == obstacle_type_rectangle)
    {
        distance_t demi_w = (obstacle->data.rectangle.width / 2 + *seg_radius);
        distance_t demi_h = (obstacle->data.rectangle.height / 2 + *seg_radius);
        points[0].x = obstacle->data.rectangle.coordinates.x - demi_w;
        points[0].y = obstacle->data.rectangle.coordinates.y - demi_h;

        points[1].x = obstacle->data.rectangle.coordinates.x + demi_w;
        points[1].y = obstacle->data.rectangle.coordinates.y - demi_h;

        points[2].x = obstacle->data.rectangle.coordinates.x + demi_w;
        points[2].y = obstacle->data.rectangle.coordinates.y + demi_h;

        points[3].x = obstacle->data.rectangle.coordinates.x - demi_w;
        points[3].y = obstacle->data.rectangle.coordinates.y + demi_h;
        sides_to_check = 4;
    }
    else if (obstacle->type == obstacle_type_circle)
    {
        const float step = 2 * M_PI / OBSTACLE_COLLISION_NB_MAX_SIDES;
        float radius = (obstacle->data.circle.radius + *seg_radius);

        for (size_t i = 0; i < OBSTACLE_COLLISION_NB_MAX_SIDES; i++)
        {
            points[i].x = obstacle->data.circle.coordinates.x + radius * cosf(i * step);
            points[i].y = obstacle->data.circle.coordinates.y + radius * sinf(i * step);
        }
        sides_to_check = OBSTACLE_COLLISION_NB_MAX_SIDES;
    }
    else
    {
        return OBSTACLE_COLLISION_ERROR_UNSUPPORTED;
    }

    coordinates_t out_pt_coll[OBSTACLE_COLLISION_NB_MAX_SIDES];
    uint8_t nb_coll = 0;
    uint8_t is_colliding = check_seg_collision(start_point, end_point, &points[0], &points[sides_to_check - 1], &out_pt_coll[nb_coll]);
    nb_coll += is_colliding;
    for (size_t i = 1; i < sides_to_check; i++)
    {
        is_colliding = check_seg_collision(start_point, end_point, &points[i - 1], &points[i], &out_pt_coll[nb_coll]);
        nb_coll += is_colliding;
    }

    if (!nb_coll)
    {
        *out_crd = *end_point;
        return 0;
    }

    distance_t closest_dist = UINT16_MAX;
    for (size_t i = 0; i < nb_coll; i++)
    {
        distance_t dist = utils_distance(&out_pt_coll[i], start_point);
        if (dist <= closest_dist)
        {
            closest_dist = dist;
            *out_crd = out_pt_coll[i];
        }
    }

    return 1; // NO COLLISION
}
