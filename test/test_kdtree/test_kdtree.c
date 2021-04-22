#include <unity.h>
#include "kdtree.h"
#include "stdlib.h"

kd_tree_t * main_tree;

void setUp(void)
{
    main_tree = kdtree_create_equally_spaced(3, 3000, 2000, sizeof(nodes_holder_t));
}

void tearDown(void)
{
    kdtree_delete(main_tree);
}

void test_init_kdtree(void)
{
    kd_tree_t *tree = kdtree_create_equally_spaced(3, 3000, 2000, sizeof(nodes_holder_t));
    TEST_ASSERT_NOT_NULL(tree);
    kdtree_print(tree, kdtree_leaf_data_print_nodes_holder);
}

void test_push_kdtree(void)
{
    path_node_t node = {0};
    node.coordinate.x = 1600;
    node.coordinate.y = 600;

    kdtree_print(main_tree, kdtree_leaf_data_print_nodes_holder);
    kdtree_push(main_tree, &node);
    kdtree_print(main_tree, kdtree_leaf_data_print_nodes_holder);

    nodes_holder_t *obj = main_tree->root->sons.block_r->sons.block_l->data;
    // TEST_ASSERT_EQUAL_MEMORY()
    // FIXME: DIM NODE 2 PAS BON

} 

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_kdtree);
    RUN_TEST(test_push_kdtree);
    return UNITY_END();
}