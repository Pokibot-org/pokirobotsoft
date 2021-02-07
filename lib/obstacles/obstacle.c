#include "obstacle.h"

int16_t obstacle_holder_get_number_of_obstacles(obstacle_holder_t *obj){
    uint16_t res = 0;
    for (uint16_t i = 0; i < OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE; i++)
    {
        if (obj->obstacles[i].type != obstacle_type_none){
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

uint8_t obstacle_get_point_of_collision_with_segment(coordinates_t *start_point, coordinates_t *end_point, obstacle_t *obstacle, uint16_t *seg_diameter, coordinates_t *out_crd)
{
    *out_crd = *start_point;
    obstacle_t fake_obs = {0};
    fake_obs.type = obstacle_type_circle;
    fake_obs.data.circle.coordinates.x = start_point->x;
    fake_obs.data.circle.coordinates.y = start_point->y;
    fake_obs.data.circle.diameter = *seg_diameter; // Precision, smaller is better but slower

    if (obstacle->type != obstacle_type_none)
    {
        uint32_t len_seg = utils_distance(*start_point, *end_point);
        uint16_t steps = len_seg * 10 / (fake_obs.data.circle.diameter);
        if (!steps)
        {
            return 2; // TODO: is the other check to do ?
        }
        int16_t step_x = (end_point->x - start_point->x) / steps;
        int16_t step_y = (end_point->y - start_point->y) / steps;
        uint8_t collision_happend = 0;
        for (uint16_t i = 0; i <= steps; i++)
        {
            if (obstacle_are_they_colliding(&fake_obs, obstacle))
            {
                collision_happend = 1;
                break;
            }
            *out_crd = fake_obs.data.circle.coordinates;
            //increment;
            fake_obs.data.circle.coordinates.x += step_x;
            fake_obs.data.circle.coordinates.y += step_y;
        }
        if (!collision_happend)
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }
    else
    {
        return OBSTACLE_COLLISION_ERROR_UNSUPPORTED;
    }

    return 0; // NO COLLISION
}
