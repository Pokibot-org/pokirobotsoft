#ifndef KDTREE_H
#define KDTREE_H
#include "stdint.h"
#include "utils.h"

typedef struct points_holder
{
    coordinates_t * points;
    uint16_t nb_of_points;
}points_holder_t;

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
    kd_superblock_t * superblocks;
}kd_tree_t;


kd_tree_t kdtree_static_create_with_points_holder();

#endif