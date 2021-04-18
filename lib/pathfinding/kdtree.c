#include "kdtree.h"
#include "string.h"
#ifdef CONFIG_KERNEL_BIN_NAME
#include "zephyr.h"
#include "logging/log.h"
LOG_MODULE_REGISTER(kdtree_lib);
#define KDTREE_PRINT_ERR(format, ...) LOG_ERR(format, ##__VA_ARGS__)
#define KDTREE_PRINT_INF(format, ...) LOG_INF(format, ##__VA_ARGS__)

K_HEAP_DEFINE(kdtree_heap, 2048);
#define KDTREE_ALLOC(size) k_heap_alloc(&kdtree_heap, size, K_NO_WAIT)
#define KDTREE_FREE(ptr) k_heap_free(&kdtree_heap, ptr)

#else
#include "stdio.h"
#include "stdlib.h"
#define KDTREE_ALLOC(size) malloc(size)
#define KDTREE_FREE(ptr) free(ptr)
#define KDTREE_PRINT_ERR(format, ...) printf("ERR>" format "\n", ##__VA_ARGS__)
#define KDTREE_PRINT_INF(format, ...) printf("INF>" format "\n", ##__VA_ARGS__)

#endif

#define KDTREE_MAX_DEEPTH 10


kd_tree_t *kdtree_create_equally_spaced(uint8_t power_of_division, distance_t width, distance_t height)
{
    if (power_of_division >= KDTREE_MAX_DEEPTH)
    {
        KDTREE_PRINT_ERR("Called with a too high power_of_division, max %d", KDTREE_MAX_DEEPTH);
        return NULL;
    }

    kd_tree_t *new_tree = KDTREE_ALLOC(sizeof(kd_tree_t));
    memset(new_tree, 0, sizeof(new_tree));

    if (new_tree == NULL)
    {
        KDTREE_PRINT_ERR("Cant alloc the tree");
        kdtree_delete(new_tree);
        return NULL;
    }

    kd_superblock_t **parents_superblock[KDTREE_MAX_DEEPTH] = {0};
    uint8_t last_free_parent = 0;
    kd_superblock_t **current_superblock = &new_tree->root;
    int8_t deepth = 0;
    float current_dims[2] = {width, height};

    while (1)
    {
        KDTREE_PRINT_INF("Deepth: %d", deepth);
        if (*current_superblock == NULL)
        {
            KDTREE_PRINT_INF("ADD superblock");
            *current_superblock = KDTREE_ALLOC(sizeof(kd_superblock_t));
            memset(*current_superblock, 0, sizeof(kd_superblock_t));
            if (*current_superblock == NULL)
            {
                KDTREE_PRINT_ERR("Cant alloc the supernode");
                kdtree_delete(new_tree);
                return NULL;
            }
            float *current_dim;
            const uint8_t nb_of_dims = 2;

            uint8_t dim_selector = deepth % nb_of_dims;
            current_dim = &current_dims[dim_selector];
            
            *current_dim = *current_dim / 2;
            (*current_superblock)->dim = *current_dim;

            KDTREE_PRINT_INF("ADD BLOCK with dim %d", (*current_superblock)->dim);
        }

        // IF LEAF
        if (deepth >= power_of_division - 1)
        {
            KDTREE_PRINT_INF("On leaf");
            (*current_superblock)->data = KDTREE_ALLOC(sizeof(nodes_holder_t));
            memset((*current_superblock)->data, 0, sizeof(nodes_holder_t));

            if ((*current_superblock)->data == NULL)
            {
                KDTREE_PRINT_ERR("Cant alloc the leaf data");
                kdtree_delete(new_tree);
                return NULL;
            }
        }
        else if ((*current_superblock)->sons.block_l == NULL)
        {
            KDTREE_PRINT_INF("Go to son l");
            deepth += 1;
            parents_superblock[last_free_parent] = current_superblock;
            last_free_parent += 1;
            current_superblock = &(*current_superblock)->sons.block_l;
            continue;
        }
        else if ((*current_superblock)->sons.block_r == NULL)
        {
            KDTREE_PRINT_INF("Go to son r");
            deepth += 1;
            parents_superblock[last_free_parent] = current_superblock;
            last_free_parent += 1;
            current_superblock = &(*current_superblock)->sons.block_r;
            continue;
        }

        KDTREE_PRINT_INF("Go up");
        deepth -= 1;
        if (deepth < 0)
        {
            break;
        }
        *current_dims = *current_dims * 2;
        last_free_parent -= 1;
        current_superblock = parents_superblock[last_free_parent];
    }

    return new_tree;
}


void kdtree_clear_leafs(kd_tree_t *tree){

};


void kdtree_delete(kd_tree_t *tree){

};


uint8_t kdtree_push(kd_tree_t *tree, const point_t *point){

};
