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
    kd_tree_t *tree = kdtree_create_equally_spaced(3, 3000, 2000);
    TEST_ASSERT_NOT_NULL(tree);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_kdtree);
    return UNITY_END();
}