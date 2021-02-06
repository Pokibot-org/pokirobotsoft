#include "pathfinding.h"
#include "stdio.h"
#include "pathfinding_utils.h"
#include "stdlib.h"

#define DEBUG_TAB_SIZE_X 120
#define DEBUG_TAB_SIZE_Y 40

int pathfinding_object_configure(pathfinding_object_t *obj, pathfinding_configuration_t *config)
{
    obj->config = config;
    if ((obj->config->field_boundaries.x == obj->config->field_boundaries.y) && (obj->config->field_boundaries.x == 0))
    {
        obj->config->field_boundaries = (boundaries_t){UINT16_MAX, UINT16_MAX};
    }
    if (!obj->config->distance_to_destination)
    {
        obj->config->distance_to_destination = (config->field_boundaries.x + config->field_boundaries.y) / 200;
    }
    if (!obj->config->delta_distance)
    {
        obj->config->delta_distance = obj->config->distance_to_destination * 2;
    }

    return PATHFINDING_ERROR_NONE;
}

path_node_t *get_closest_node(pathfinding_object_t *obj, coordinates_t *crd)
{
    path_node_t *closest_node_p = NULL;
    uint32_t closest_node_distance = UINT32_MAX;

    for (size_t i = 0; i < PATHFINDING_MAX_NUM_OF_NODES; i++)
    {
        if (obj->nodes[i].is_used)
        {
            uint32_t distance = utils_distance(obj->nodes[i].coordinate, *crd);
            if (distance < closest_node_distance)
            {
                closest_node_distance = distance;
                closest_node_p = &obj->nodes[i];
            }
        }
    }
    return closest_node_p;
}

int get_new_valid_coordinates(pathfinding_object_t *obj, coordinates_t *crd_tree_node, coordinates_t *crd_random_node, coordinates_t *crd_new_node)
{
    if (utils_distance(*crd_tree_node, *crd_random_node) <= obj->config->delta_distance)
    {
        *crd_new_node = *crd_random_node;
        return PATHFINDING_ERROR_NONE;
    }

    vector_t vector = {
        .x = crd_random_node->x - crd_tree_node->x,
        .y = crd_random_node->y - crd_tree_node->y};
    uint32_t max_vec = utils_max(vector.x, vector.y);
    crd_new_node->x = crd_tree_node->x + vector.x * obj->config->delta_distance / max_vec;
    crd_new_node->y = crd_tree_node->y + vector.y * obj->config->delta_distance / max_vec;
    return PATHFINDING_ERROR_NONE;
}

int pathfinding_find_path(pathfinding_object_t *obj, coordinates_t *start, coordinates_t *end)
{
    // TODO: Check input validity, must be between 0 and pathfinding_boundaries
    // Init start node
    //TODO :must be bound to current tree if existing
    obj->nodes[0].coordinate = *start;
    obj->nodes[0].parent_node = NULL;
    obj->nodes[0].son_node = NULL;
    obj->nodes[0].is_used = 1;

    for (size_t i = 1; i < PATHFINDING_MAX_NUM_OF_NODES; i++)
    {
        path_node_t *current_node = &obj->nodes[i];
        if (current_node->is_used)
        {
            // ?
        }
        else
        {
            coordinates_t rand_coordinates;
            rand_coordinates.x = utils_get_rand32() % obj->config->field_boundaries.x;
            rand_coordinates.y = utils_get_rand32() % obj->config->field_boundaries.y;
            path_node_t *closest_node_p = get_closest_node(obj, &rand_coordinates);
            // check if collision
            coordinates_t new_coordinates;
            get_new_valid_coordinates(obj, &(closest_node_p->coordinate), &rand_coordinates, &new_coordinates);
            current_node->is_used = 1;
            current_node->coordinate = new_coordinates;
            current_node->parent_node = closest_node_p;
            closest_node_p->son_node = current_node;
            // printf("New node<x:%d,y%d>\n", new_coordinates.x, new_coordinates.y);
        }
    }

    return PATHFINDING_ERROR_PATH_NOT_FOUND;
}

void pathfinding_debug_print(pathfinding_object_t *obj)
{
    uint8_t tab[DEBUG_TAB_SIZE_Y][DEBUG_TAB_SIZE_X] = {0};
    for (size_t i = 0; i < PATHFINDING_MAX_NUM_OF_NODES; i++)
    {
        if (obj->nodes[i].is_used)
        {
            uint16_t y = obj->nodes[i].coordinate.y * DEBUG_TAB_SIZE_Y / obj->config->field_boundaries.y;
            uint16_t x = obj->nodes[i].coordinate.x * DEBUG_TAB_SIZE_X / obj->config->field_boundaries.x;
            // printf("%d %d | %d %d\n", y, x, obj->nodes[i].coordinate.y, obj->nodes[i].coordinate.x);
            tab[y][x] = 1;
        }
    }
    // printf("FB: %d %d\n", obj->config->field_boundaries.x, obj->config->field_boundaries.y);
    for (size_t y = 0; y < DEBUG_TAB_SIZE_Y; y++)
    {
        for (size_t x = 0; x < DEBUG_TAB_SIZE_X; x++)
        {
            char c = tab[y][x] ? 'X' : '.';
            printf("%c", c);
        }
        printf("\n");
    }
};