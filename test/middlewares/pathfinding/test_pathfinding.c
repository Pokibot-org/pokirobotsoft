#include <unity.h>
#include "pathfinding.h"
#include "stdlib.h"
#include "time.h"

void setUp(void) {
    pathfinding_configuration_t config;
    config.field_boundaries.x = 2000;
    config.field_boundaries.y = 3000;
    pathfinding_configure(&config);
    srand(time(NULL));
}

void tearDown(void) {
    // clean stuff up here
}

void test_in_free_space_path_must_be_found(void) {
    coordinates_t start = {
        .x = 10,
        .y = 10,
    };
    coordinates_t end = {
        .x = 1000,
        .y = 1000,
    };
    TEST_ASSERT_EQUAL(0, pathfinding_find_path(&start, &end));
    pathfinding_debug_print();
}


int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_in_free_space_path_must_be_found);
    return UNITY_END();
}