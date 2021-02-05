#include "pathfinding.h"
#include "stdio.h"
#include "pathfinding_utils.h"
#include "stdlib.h"

#define DEBUG_TAB_SIZE 80

static path_node_t nodes[PATHFINDING_MAX_NUM_OF_NODES] = {0};
static boundaries_t pathfinding_boundaries = {
    .x = PATHFINDING_MAX_BOUNDARIES,
    .y = PATHFINDING_MAX_BOUNDARIES};
static uint16_t distance_to_destination = {0};
static uint16_t delta_distance = {40};

int pathfinding_configure(pathfinding_configuration_t *config)
{
    // TODO: test not field_boundaries over PATHFINDING_MAX_BOUNDARIES
    pathfinding_boundaries = config->field_boundaries;
    // distance_to_destination = 1% of the area size
    distance_to_destination = (config->field_boundaries.x + config->field_boundaries.y) / 200;
    return 0;
}

path_node_t *get_closest_node(coordinates_t *crd)
{
    path_node_t *closest_node_p = NULL;
    uint32_t closest_node_distance = UINT32_MAX;

    for (size_t i = 0; i < PATHFINDING_MAX_NUM_OF_NODES; i++)
    {
        if (nodes[i].is_used)
        {
            uint32_t distance = utils_distance(nodes[i].coordinate, *crd);
            if (distance < closest_node_distance)
            {
                closest_node_distance = distance;
                closest_node_p = &nodes[i];
            }
        }
    }
    return closest_node_p;
}

void get_new_valid_coordinates(coordinates_t *crd_tree_node, coordinates_t *crd_random_node, coordinates_t *crd_new_node)
{
    if (utils_distance(*crd_tree_node, *crd_random_node) <= delta_distance)
    {
        *crd_new_node = *crd_random_node;
        return;
    }

    vector_t vector = {
        .x = crd_random_node->x - crd_tree_node->x,
        .y = crd_random_node->y - crd_tree_node->y};
    uint32_t max_vec = utils_max(vector.x, vector.y);
    crd_new_node->x = crd_tree_node->x + vector.x * delta_distance / max_vec;
    crd_new_node->y = crd_tree_node->y + vector.y * delta_distance / max_vec;
}

int pathfinding_find_path(coordinates_t *start, coordinates_t *end)
{
    // Init start node
    //TODO :must be bound to current tree if existing
    nodes[0].coordinate = *start;
    nodes[0].parent_node = NULL;
    nodes[0].son_node = NULL;
    nodes[0].is_used = 1;

    for (size_t i = 1; i < PATHFINDING_MAX_NUM_OF_NODES; i++)
    {
        path_node_t *current_node = &nodes[i];
        if (current_node->is_used)
        {
            // ?
        }
        else
        {
            coordinates_t rand_coordinates;
            rand_coordinates.x = utils_get_rand32() % pathfinding_boundaries.x;
            rand_coordinates.y = utils_get_rand32() % pathfinding_boundaries.y;
            path_node_t *closest_node_p = get_closest_node(&rand_coordinates);
            // check if collision
            coordinates_t new_coordinates;
            get_new_valid_coordinates(&(closest_node_p->coordinate), &rand_coordinates, &new_coordinates);
            current_node->is_used = 1;
            current_node->coordinate = new_coordinates;
            current_node->parent_node = closest_node_p;
            closest_node_p->son_node = current_node;
            // printf("New node<x:%d,y%d>\n", new_coordinates.x, new_coordinates.y);
        }
    }

    return 0;
}

void pathfinding_debug_print()
{
    uint8_t tab[DEBUG_TAB_SIZE][DEBUG_TAB_SIZE] = {0};
    for (size_t i = 0; i < PATHFINDING_MAX_NUM_OF_NODES; i++)
    {
        if (nodes[i].is_used)
        {
            uint16_t y = nodes[i].coordinate.y * DEBUG_TAB_SIZE / pathfinding_boundaries.y;
            uint16_t x = nodes[i].coordinate.x * DEBUG_TAB_SIZE / pathfinding_boundaries.x;
            printf("%d %d | %d %d\n", y, x, nodes[i].coordinate.y, nodes[i].coordinate.x);
            tab[y][x] = 1;
        }
    }

    for (size_t y = 0; y < DEBUG_TAB_SIZE; y++)
    {
        for (size_t x = 0; x < DEBUG_TAB_SIZE; x++)
        {
            char c = tab[y][x] ? 'X' : '.';
            printf("%c", c);
        }
        printf("\n");
    }
};