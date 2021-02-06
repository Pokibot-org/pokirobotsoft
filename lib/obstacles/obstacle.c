#include "obstacle.h"

uint8_t are_rectangle_and_circle_colliding(rectangle_t *rec, circle_t *cir)
{
    uint32_t circle_distance_x = ABS((int32_t)cir->coordinates.x - (int32_t)rec->coordinates.x);
    uint32_t circle_distance_y = ABS((int32_t)cir->coordinates.y - (int32_t)rec->coordinates.y);

    if (circle_distance_x > ((rec->width + cir->diameter) / 2))
    {
        return 0;
    }
    if (circle_distance_y > ((rec->height + cir->diameter) / 2))
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

    return (corner_distance_sq <= SQUARE(cir->diameter / 2));
}

uint8_t obstacle_are_they_colliding(obstacle_t *a, obstacle_t *b)
{
    if ((a->type == obstacle_type_none) || (b->type == obstacle_type_none))
    {
        return 255;
    }
    if (a->type == b->type)
    {
        if (a->type == obstacle_type_circle)
        {
            return utils_distance(a->data.circle.coordinates, b->data.circle.coordinates) <= ((a->data.circle.diameter + b->data.circle.diameter) / 2);
        }
        else
        {
            return 255; // FIXME: NOT SUPPORTED
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
    return 255;
}