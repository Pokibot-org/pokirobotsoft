#include <unity.h>
#include "pathfinding.h"
#include "stdlib.h"
#include "time.h"
#include "stdio.h"
#include "string.h"
#include "obstacle.h"

#ifndef M_SQRT2
#define M_SQRT2 1.41421356237309504880f
#endif

// #define FORCE_PRINT

#ifdef FORCE_PRINT
#define FORCE_PRINT_CONSTANT 1
#else
#define FORCE_PRINT_CONSTANT 0
#endif

// #define FORCE_RANDOM

FILE *psr_fd;
pathfinding_object_t pathfinding_obj;

void setUp(void)
{
    pathfinding_configuration_t config;
    config.field_boundaries.max_x = 3000; // 3m
    config.field_boundaries.max_y = 2000; // 2m
    config.field_boundaries.min_x = 0;
    config.field_boundaries.min_y = 0;
    config.delta_distance = 400;     // jump of Xcm
    config.radius_of_security = 150; // Basicaly the robot radius in mm
    memset(&pathfinding_obj.nodes, 0, PATHFINDING_MAX_NUM_OF_NODES * sizeof(path_node_t));
    pathfinding_object_configure(&pathfinding_obj, &config);

    psr_fd = fopen("./psr.txt", "a+");
#ifdef FORCE_RANDOM
    srand(time(NULL));
    uint32_t tab[4];
    for (size_t i = 0; i < 4; i++)
    {
        tab[i] = rand();
    }
    utils_init_rand_seed(tab);
#endif
}

void tearDown(void)
{
    fclose(psr_fd);
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
    if (FORCE_PRINT_CONSTANT || err)
    {
        pathfinding_debug_print(&pathfinding_obj);
    }
    TEST_ASSERT_EQUAL(PATHFINDING_ERROR_NONE, err);
    if (FORCE_PRINT_CONSTANT || err)
    {
        pathfinding_debug_print_found_path(&pathfinding_obj, end_node);
        printf("Found in %d nodes!\n", pathfinding_get_number_of_used_nodes(&pathfinding_obj));
    }
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
    if (FORCE_PRINT_CONSTANT || err)
    {
        pathfinding_debug_print(&pathfinding_obj);
    }
    TEST_ASSERT_EQUAL(PATHFINDING_ERROR_NONE, err);
    if (FORCE_PRINT_CONSTANT || err)
    {
        pathfinding_debug_print_found_path(&pathfinding_obj, end_node);
        printf("Found in %d nodes!\n Now optimizing path : \n", pathfinding_get_number_of_used_nodes(&pathfinding_obj));
        pathfinding_optimize_path(&pathfinding_obj, &ob_hold, end_node, PATHFINDING_MAX_NUM_OF_NODES);
        pathfinding_debug_print_found_path(&pathfinding_obj, end_node);
    }
}

void test_with_obstacle_path_must_be_found_hard_config(void)
{
    obstacle_holder_t ob_hold = {0};
    obstacle_t rec = {
        .type = obstacle_type_rectangle,
        .data.rectangle = {
            .coordinates = {
                .x = 600,
                .y = 500},
            .height = 1000,
            .width = 200,
        }};
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
    if (FORCE_PRINT_CONSTANT || err)
    {
        pathfinding_debug_print(&pathfinding_obj);
    }
    TEST_ASSERT_EQUAL(PATHFINDING_ERROR_NONE, err);
    if (FORCE_PRINT_CONSTANT || err)
    {
        pathfinding_debug_print_found_path(&pathfinding_obj, end_node);
        float time_spent = (float)(end_clk - begin_clk) / CLOCKS_PER_SEC * 1000;
        printf("Found in %d nodes! Time : %f ms | Len : %d\n Now optimizing: \n", pathfinding_get_number_of_used_nodes(&pathfinding_obj), time_spent, end_node->distance_to_start);
        begin_clk = clock();
        pathfinding_optimize_path(&pathfinding_obj, &ob_hold, end_node, PATHFINDING_MAX_NUM_OF_NODES);
        end_clk = clock();
        pathfinding_debug_print_found_path(&pathfinding_obj, end_node);

        time_spent = (float)(end_clk - begin_clk) / CLOCKS_PER_SEC * 1000;
        printf("Optimized with total of %d nodes! Time : %f ms | Len : %d\n Now optimizing: \n", pathfinding_get_number_of_used_nodes(&pathfinding_obj), time_spent, end_node->distance_to_start);
        // pathfinding_debug_write_found_path_list(&pathfinding_obj, end_node, "/tmp/path");
    }
}

void test_with_lidar_obstacle_path_must_be_found(void)
{
    obstacle_holder_t ob_hold = {0};
    obstacle_t obs = {
        .type = obstacle_type_circle,
        .data.circle = {
            .coordinates = {
                .x = 1500,
                .y = 10},
            .radius = 0}};
    for (size_t i = 0; i < 60; i++)
    {
        obstacle_holder_push(&ob_hold, &obs);
        obs.data.circle.coordinates.y += 20;
    }
    obs.data.circle.coordinates.x = 2000;
    obs.data.circle.coordinates.y = 2000;
    for (size_t i = 0; i < 60; i++)
    {
        obstacle_holder_push(&ob_hold, &obs);
        obs.data.circle.coordinates.y -= 20;
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
    float time_spent = (float)(end_clk - begin_clk) / CLOCKS_PER_SEC * 1000;
    if (err)
    {
        fprintf(psr_fd, "Path not found\n");
    }
    else
    {
        fprintf(psr_fd, "Found in %d nodes! Time : %f ms | Len : %d\n", pathfinding_get_number_of_used_nodes(&pathfinding_obj), time_spent, end_node->distance_to_start);
    }
    if (FORCE_PRINT_CONSTANT || err)
    {
        pathfinding_debug_print(&pathfinding_obj);
    }
    TEST_ASSERT_EQUAL(PATHFINDING_ERROR_NONE, err);
    if (FORCE_PRINT_CONSTANT || err)
    {
        pathfinding_debug_print_found_path(&pathfinding_obj, end_node);
        printf("Found in %d nodes! Time : %f ms | Len : %d\n Now optimizing: \n", pathfinding_get_number_of_used_nodes(&pathfinding_obj), time_spent, end_node->distance_to_start);
        begin_clk = clock();
        pathfinding_optimize_path(&pathfinding_obj, &ob_hold, end_node, PATHFINDING_MAX_NUM_OF_NODES);
        end_clk = clock();
        pathfinding_debug_print_found_path(&pathfinding_obj, end_node);

        time_spent = (float)(end_clk - begin_clk) / CLOCKS_PER_SEC * 1000;
        printf("Optimized with total of %d nodes! Time : %f ms | Len : %d\n Now optimizing: \n", pathfinding_get_number_of_used_nodes(&pathfinding_obj), time_spent, end_node->distance_to_start);
    }
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
    must_be_crd = (coordinates_t){10 + pathfinding_obj.config.delta_distance * M_SQRT2 / 2,
                                  10 + pathfinding_obj.config.delta_distance * M_SQRT2 / 2};
    TEST_ASSERT_EQUAL(0, get_new_valid_coordinates(&pathfinding_obj, &start, &end, &new));
    TEST_ASSERT_EQUAL_MESSAGE(must_be_crd.y, new.y, "Y");
    TEST_ASSERT_EQUAL_MESSAGE(must_be_crd.x, new.x, "X");

    // DIAGONAL 2
    start = (coordinates_t){500, 500};
    end = (coordinates_t){10, 10};
    must_be_crd = (coordinates_t){500 - pathfinding_obj.config.delta_distance * M_SQRT2 / 2,
                                  500 - pathfinding_obj.config.delta_distance * M_SQRT2 / 2};
    TEST_ASSERT_EQUAL(0, get_new_valid_coordinates(&pathfinding_obj, &start, &end, &new));
    TEST_ASSERT_TRUE_MESSAGE(new.y > must_be_crd.y * 0.99 && new.y < must_be_crd.y * 1.01, "Diag 2 y"); // not perfectly precise
    TEST_ASSERT_TRUE_MESSAGE(new.x > must_be_crd.x * 0.99 && new.x < must_be_crd.x * 1.01, "Diag 2 x");
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