#include <unity.h>
#include "pathfinding.h"
#include "stdlib.h"
#include "time.h"
#include "stdio.h"
#include "string.h"

//#define PRINT_DEBUG

pathfinding_object_t pathfinding_obj;

void setUp(void)
{
    pathfinding_configuration_t config;
    config.field_boundaries.x = 3000;
    config.field_boundaries.y = 2000;
    config.delta_distance = 80;
    config.distance_to_destination = 100;
    memset(&pathfinding_obj.nodes, 0, PATHFINDING_MAX_NUM_OF_NODES * sizeof(path_node_t));
    pathfinding_object_configure(&pathfinding_obj, &config);
    srand(time(NULL));
}

void tearDown(void)
{
    // clean stuff up here
}

void test_get_closest_node(void)
{
}

void test_in_free_space_path_must_be_found_simple_config(void)
{
    path_node_t *end_node;
    coordinates_t start = {
        .x = pathfinding_obj.config.field_boundaries.x / 2,
        .y = pathfinding_obj.config.field_boundaries.y / 2,
    };
    coordinates_t end = {
        .x = 50,
        .y = 50,
    };
    int err = pathfinding_find_path(&pathfinding_obj, &start, &end, &end_node);
#ifdef PRINT_DEBUG
    pathfinding_debug_print(&pathfinding_obj);
#endif
    TEST_ASSERT_EQUAL(PATHFINDING_ERROR_NONE, err);
#ifdef PRINT_DEBUG
    pathfinding_debug_print_found_path(&pathfinding_obj, end_node);
#endif
}

void test_in_free_space_path_must_be_found_hard_config(void)
{
    path_node_t *end_node;
    coordinates_t start = {
        .x = 10,
        .y = 10,
    };
    coordinates_t end = {
        .x = pathfinding_obj.config.field_boundaries.x - 10,
        .y = 10,
    };
    int err = pathfinding_find_path(&pathfinding_obj, &start, &end, &end_node);
#ifdef PRINT_DEBUG
    pathfinding_debug_print(&pathfinding_obj);
#endif
    TEST_ASSERT_EQUAL(PATHFINDING_ERROR_NONE, err);
#ifdef PRINT_DEBUG
    pathfinding_debug_print_found_path(&pathfinding_obj, end_node);
#endif
}

void test_get_new_valid_coordinates()
{
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
        .y = 10 + pathfinding_obj.config.delta_distance};
    coordinates_t new;
    TEST_ASSERT_EQUAL(0, get_new_valid_coordinates(&pathfinding_obj, &start, &end, &new));
    TEST_ASSERT_EQUAL_MESSAGE(must_be_crd.y, new.y, "Y");
    TEST_ASSERT_EQUAL_MESSAGE(must_be_crd.x, new.x, "X");

    // DIAGONAL
    end = (coordinates_t){1000, 1000};
    must_be_crd = (coordinates_t){10 + pathfinding_obj.config.delta_distance * M_SQRT2 / 2, 10 + pathfinding_obj.config.delta_distance * M_SQRT2 / 2};
    TEST_ASSERT_EQUAL(0, get_new_valid_coordinates(&pathfinding_obj, &start, &end, &new));
    TEST_ASSERT_EQUAL_MESSAGE(must_be_crd.y, new.y, "Y");
    TEST_ASSERT_EQUAL_MESSAGE(must_be_crd.x, new.x, "X");

    // DIAGONAL 2
    start = (coordinates_t){500, 500};
    end = (coordinates_t){10, 10};
    must_be_crd = (coordinates_t){500 - pathfinding_obj.config.delta_distance * M_SQRT2 / 2,
                                  500 - pathfinding_obj.config.delta_distance * M_SQRT2 / 2};
    TEST_ASSERT_EQUAL(0, get_new_valid_coordinates(&pathfinding_obj, &start, &end, &new));
    TEST_ASSERT_EQUAL_MESSAGE(must_be_crd.y, new.y, "Y");
    TEST_ASSERT_EQUAL_MESSAGE(must_be_crd.x, new.x, "X");
    // TO CLOSE
    start = (coordinates_t){10, 10};
    end = (coordinates_t){20, 20};
    must_be_crd = (coordinates_t){20, 20};
    TEST_ASSERT_EQUAL(0, get_new_valid_coordinates(&pathfinding_obj, &start, &end, &new));
    TEST_ASSERT_EQUAL_MESSAGE(must_be_crd.y, new.y, "Y");
    TEST_ASSERT_EQUAL_MESSAGE(must_be_crd.x, new.x, "X");
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_get_closest_node);
    RUN_TEST(test_in_free_space_path_must_be_found_simple_config);
    RUN_TEST(test_in_free_space_path_must_be_found_hard_config);
    RUN_TEST(test_get_new_valid_coordinates);
    return UNITY_END();
}