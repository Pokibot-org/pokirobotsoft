#include "pathfinding.h"
#include "stdio.h"

#define DEBUG_TAB_SIZE 80


static path_node_t nodes[PATHFINDING_MAX_NUM_OF_NODES] = {0};
static boundaries_t pathfinding_boundaries = {
    .x = PATHFINDING_MAX_BOUNDARIES,
    .y = PATHFINDING_MAX_BOUNDARIES
};

int pathfinding_configure(pathfinding_configuration_t *config){
    // TODO: test not field_boundaries over PATHFINDING_MAX_BOUNDARIES
    pathfinding_boundaries = config->field_boundaries;
    return 0;
}

int pathfinding_find_path(coordinates_t *start, coordinates_t *end){

    return 0;
}

void pathfinding_debug_print(){
    uint8_t tab[DEBUG_TAB_SIZE][DEBUG_TAB_SIZE] = {0};
    for (size_t i = 0; i < PATHFINDING_MAX_NUM_OF_NODES; i++)
    {
        if(nodes[i].is_used){
            uint16_t y = nodes[i].coordinate.y * DEBUG_TAB_SIZE / pathfinding_boundaries.y;
            uint16_t x = nodes[i].coordinate.x * DEBUG_TAB_SIZE / pathfinding_boundaries.x;
            tab[y][x] = 1;
        }
    }

    for (size_t y = 0; y < DEBUG_TAB_SIZE; y++)
    {
        for (size_t x = 0; x < DEBUG_TAB_SIZE; x++)
        {
            char c = tab[y][x] ? 'X' : ' ';
            printf("%c", c);
        }
        printf("\n");
    }
};