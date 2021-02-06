#include <unity.h>
#include "pathfinding.h"
#include "stdlib.h"
#include "time.h"

pathfinding_configuration_t config;
pathfinding_object_t pathfinding_obj;

void setUp(void) {
    config.field_boundaries.x = 2000;
    config.field_boundaries.y = 3000;
    config.delta_distance = 40;
    config.distance_to_destination = 20;
    pathfinding_object_configure(&pathfinding_obj , &config);
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
    int err = pathfinding_find_path(&pathfinding_obj, &start, &end);
    pathfinding_debug_print(&pathfinding_obj);
    TEST_ASSERT_EQUAL(PATHFINDING_ERROR_NONE, err);
}

void test_get_new_valid_coordinates(){
    coordinates_t start = {
        .x = 10,
        .y = 10,
    };
    coordinates_t end = {
        .x = 10,
        .y = 1000,
    };
    coordinates_t must_be_crd = {
        .x = 10,
        .y = 10 + pathfinding_obj.config->delta_distance
    };
    coordinates_t new;
    TEST_ASSERT_EQUAL(0, get_new_valid_coordinates(&pathfinding_obj, &start, &end, &new));
    TEST_ASSERT_EQUAL(must_be_crd.y, new.y);
    TEST_ASSERT_EQUAL(must_be_crd.x, new.x);

    // DIAGONAL
    end = (coordinates_t){1000,1000};
    must_be_crd = (coordinates_t){10 + pathfinding_obj.config->delta_distance* M_SQRT2/2,10 + pathfinding_obj.config->delta_distance* M_SQRT2/2};
    TEST_ASSERT_EQUAL(0, get_new_valid_coordinates(&pathfinding_obj, &start, &end, &new));
    TEST_ASSERT_EQUAL(must_be_crd.y, new.y);
    TEST_ASSERT_EQUAL(must_be_crd.x, new.x);

    // TO CLOSE

    end = (coordinates_t){20, 20};
    must_be_crd = (coordinates_t){20, 20};
    TEST_ASSERT_EQUAL(0, get_new_valid_coordinates(&pathfinding_obj, &start, &end, &new));
    TEST_ASSERT_EQUAL(must_be_crd.y, new.y);
    TEST_ASSERT_EQUAL(must_be_crd.x, new.x);
}


int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_in_free_space_path_must_be_found);
    RUN_TEST(test_get_new_valid_coordinates);
    return UNITY_END();
}