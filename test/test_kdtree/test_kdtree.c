#include <unity.h>
#include "kdtree.h"
#include "stdlib.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_init_kdtree(void)
{
    kd_tree_t *tree = kdtree_create_equally_spaced(3, 3000, 2000, sizeof(nodes_holder_t));
    TEST_ASSERT_NOT_NULL(tree);
    kdtree_print(tree, kdtree_leaf_data_print_nodes_holder);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_kdtree);
    return UNITY_END();
}