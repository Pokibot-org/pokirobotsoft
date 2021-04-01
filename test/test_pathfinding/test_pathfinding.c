#include <unity.h>
#include "pathfinding.h"
#include "stdlib.h"
#include "time.h"
#include "stdio.h"
#include "string.h"
#include "obstacle.h"

#ifndef M_SQRT2
#define M_SQRT2		1.41421356237309504880
#endif


#define PRINT_DEBUG


pathfinding_object_t pathfinding_obj;

void setUp(void)
{
    pathfinding_configuration_t config;
    config.field_boundaries.max_x = 3000; // 3m
    config.field_boundaries.max_y = 2000; // 2m
    config.delta_distance = 400; // jump of Xcm
    config.distance_to_destination = 60; // stop when less than 6 cm close tho goal
    config.radius_of_security = 300; // 300 mm
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
    obstacle_holder_t ob_hold = {0};
    path_node_t *end_node;
    coordinates_t start = {
        .x = pathfinding_obj.config.field_boundaries.max_x / 2,
        .y = pathfinding_obj.config.field_boundaries.max_y / 2,
    };
    coordinates_t end = {
        .x = 50,
        .y = 50,
    };
    int err = pathfinding_find_path(&pathfinding_obj, &ob_hold, &start, &end, &end_node);
#ifdef PRINT_DEBUG
    pathfinding_debug_print(&pathfinding_obj);
#endif
    TEST_ASSERT_EQUAL(PATHFINDING_ERROR_NONE, err);
#ifdef PRINT_DEBUG
    pathfinding_debug_print_found_path(&pathfinding_obj, end_node);
    printf("Found in %d nodes!\n", pathfinding_get_number_of_used_nodes(&pathfinding_obj));
#endif
}

void test_in_free_space_path_must_be_found_hard_config(void)
{
    obstacle_holder_t ob_hold = {0};
    path_node_t *end_node;
    coordinates_t start = {
        .x = 10,
        .y = 10,
    };
    coordinates_t end = {
        .x = pathfinding_obj.config.field_boundaries.max_x - 10,
        .y = 10,
    };
    int err = pathfinding_find_path(&pathfinding_obj, &ob_hold, &start, &end, &end_node);
#ifdef PRINT_DEBUG
    pathfinding_debug_print(&pathfinding_obj);
#endif
    TEST_ASSERT_EQUAL(PATHFINDING_ERROR_NONE, err);
#ifdef PRINT_DEBUG
    pathfinding_debug_print_found_path(&pathfinding_obj, end_node);
    printf("Found in %d nodes!\n Now optimizing path : \n", pathfinding_get_number_of_used_nodes(&pathfinding_obj));
    pathfinding_optimize_path(&pathfinding_obj, &ob_hold, PATHFINDING_MAX_NUM_OF_NODES);
    pathfinding_debug_print_found_path(&pathfinding_obj, end_node);
#endif
}

void test_with_obstacle_path_must_be_found_hard_config(void)
{
    obstacle_holder_t ob_hold = {0};
    obstacle_t rec = {
        .type = obstacle_type_rectangle,
        .data.rectangle = {
            .coordinates = {
                .x = 600,
                .y = 500
            },
            .height = 1000,
            .width = 200,
        }
    };
    obstacle_holder_push(&ob_hold, &rec);
    path_node_t *end_node;
    coordinates_t start = {
        .x = 10,
        .y = 10,
    };
    coordinates_t end = {
        .x = pathfinding_obj.config.field_boundaries.max_x - 10,
        .y = 10,
    };    
    clock_t begin_clk = clock();
    int err = pathfinding_find_path(&pathfinding_obj, &ob_hold, &start, &end, &end_node);
    clock_t end_clk = clock();
#ifdef PRINT_DEBUG
    pathfinding_debug_print(&pathfinding_obj);
#endif
    TEST_ASSERT_EQUAL(PATHFINDING_ERROR_NONE, err);
#ifdef PRINT_DEBUG
    pathfinding_debug_print_found_path(&pathfinding_obj, end_node);
    float time_spent = (float)(end_clk - begin_clk) / CLOCKS_PER_SEC * 1000;
    printf("Found in %d nodes! Time : %f ms | Len : %d\n Now optimizing: \n", pathfinding_get_number_of_used_nodes(&pathfinding_obj), time_spent, end_node->distance_to_start);
    begin_clk = clock();
    pathfinding_optimize_path(&pathfinding_obj, &ob_hold, PATHFINDING_MAX_NUM_OF_NODES);
    end_clk = clock();
    pathfinding_debug_print_found_path(&pathfinding_obj, end_node);

    time_spent = (float)(end_clk - begin_clk) / CLOCKS_PER_SEC * 1000;
    printf("Optimized with total of %d nodes! Time : %f ms | Len : %d\n Now optimizing: \n", pathfinding_get_number_of_used_nodes(&pathfinding_obj), time_spent, end_node->distance_to_start);
    // pathfinding_debug_write_found_path_list(&pathfinding_obj, end_node, "/tmp/path");
#endif
}


void test_with_lidar_obstacle_path_must_be_found(void)
{
    obstacle_holder_t ob_hold = {0};
    obstacle_t obs = {
        .type = obstacle_type_circle,
        .data.circle = {
            .coordinates = {
                .x = 1500,
                .y = 10
            },
            .diameter = 0
        }
    };
    for (size_t i = 0; i < 60; i++)
    {
        obstacle_holder_push(&ob_hold, &obs);
        obs.data.circle.coordinates.y += 20;
    }
    
    path_node_t *end_node;
    coordinates_t start = {
        .x = 40,
        .y = 40,
    };
    coordinates_t end = {
        .x = pathfinding_obj.config.field_boundaries.max_x - 40,
        .y = 500,
    };    
    clock_t begin_clk = clock();
    int err = pathfinding_find_path(&pathfinding_obj, &ob_hold, &start, &end, &end_node);
    clock_t end_clk = clock();
#ifdef PRINT_DEBUG
    pathfinding_debug_print(&pathfinding_obj);
#endif
    TEST_ASSERT_EQUAL(PATHFINDING_ERROR_NONE, err);
#ifdef PRINT_DEBUG
    pathfinding_debug_print_found_path(&pathfinding_obj, end_node);
    float time_spent = (float)(end_clk - begin_clk) / CLOCKS_PER_SEC * 1000;
    printf("Found in %d nodes! Time : %f ms | Len : %d\n Now optimizing: \n", pathfinding_get_number_of_used_nodes(&pathfinding_obj), time_spent, end_node->distance_to_start);
    begin_clk = clock();
    pathfinding_optimize_path(&pathfinding_obj, &ob_hold, PATHFINDING_MAX_NUM_OF_NODES);
    end_clk = clock();
    pathfinding_debug_print_found_path(&pathfinding_obj, end_node);

    time_spent = (float)(end_clk - begin_clk) / CLOCKS_PER_SEC * 1000;
    printf("Optimized with total of %d nodes! Time : %f ms | Len : %d\n Now optimizing: \n", pathfinding_get_number_of_used_nodes(&pathfinding_obj), time_spent, end_node->distance_to_start);
    // pathfinding_debug_write_found_path_list(&pathfinding_obj, end_node, "/tmp/path");
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
    RUN_TEST(test_with_obstacle_path_must_be_found_hard_config);
    RUN_TEST(test_with_lidar_obstacle_path_must_be_found);
    RUN_TEST(test_get_new_valid_coordinates);
    return UNITY_END();
}