#include "obstacle.h"

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
        obj->most_left_known_free_item = head;
    }
}

uint8_t obstacle_holder_push(obstacle_holder_t *obj, obstacle_t *obstacle)
{
    if (obj->most_left_known_free_item == OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE)
    {
        obstacle_holder_compact(obj);
        if (obj->most_left_known_free_item == OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE)
        {
            return OBSTACLE_HOLDER_ERROR_TO_FULL;
        }
    }
    obj->obstacles[obj->most_left_known_free_item] = *obstacle;
}

uint8_t obstacle_holder_delete_index(obstacle_holder_t *obj, uint16_t index)
{
    if (index >= OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE)
    {
        return OBSTACLE_HOLDER_ERROR_INVALID_INDEX;
    }
    obj->obstacles[index].type = obstacle_type_none;
}

/**
 * @brief Delete the obtacle object from the obstacle_holder object.
 * THE OBSTACLE OBJECT ADRESS MUST BE IN THE obstacle_holder_t OBJECT
 */
uint8_t obstacle_holder_delete(obstacle_holder_t *obj, obstacle_t *obstacle)
{
    uint32_t index = (obstacle - obj->obstacles) / sizeof(obstacle_t);
    return obstacle_holder_delete_index(obj, index);
}

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
        return OBSTACLE_COLLISION_ERROR_UNSUPPORTED;
    }
    if (a->type == b->type)
    {
        if (a->type == obstacle_type_circle)
        {
            return utils_distance(a->data.circle.coordinates, b->data.circle.coordinates) <= ((a->data.circle.diameter + b->data.circle.diameter) / 2);
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