#ifndef KDTREE_H
#define KDTREE_H
#include "stdint.h"
#include "robot_utils.h"
#include "pathfinding_types.h"

#define KDTREE_POINT_HOLDER_LEN 20

typedef struct nodes_holder
{
    path_node_t points[KDTREE_POINT_HOLDER_LEN];
    uint16_t nb_of_nodes;
}nodes_holder_t;

typedef struct kd_superblock
{
    struct sons
    {
        struct kd_superblock * block_l;
        struct kd_superblock * block_r;
    }sons;
    void * data;
    int32_t dim;
}kd_superblock_t;

typedef struct kd_tree
{
    kd_superblock_t * root;
}kd_tree_t;

typedef void (*kdtree_leaf_data_print)(const void * leaf_data, char * parent_node_name);  


kd_tree_t * kdtree_create_equally_spaced(uint8_t power_of_division, distance_t width, distance_t height, uint32_t leaf_size);
void kdtree_clear_leafs(kd_tree_t * tree);
void kdtree_delete(kd_tree_t * tree);
uint8_t kdtree_push(kd_tree_t *tree, const path_node_t *node);
void kdtree_print(kd_tree_t *tree, kdtree_leaf_data_print leaf_clbk);
void kdtree_leaf_data_print_nodes_holder(const void * leaf_data, char * parent_node_name);
#endif