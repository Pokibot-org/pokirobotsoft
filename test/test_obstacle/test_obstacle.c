#include <unity.h>
#include "obstacle.h"
#include "relative_obstacle_storing.h"
#include "stdio.h"


void setUp(void)
{
}

void tearDown(void)
{
    // clean stuff up here
}

void test_collision_circles(void)
{
    obstacle_t a = {
        .type = obstacle_type_circle,
        .data.circle = {
            .coordinates = {
                .x = 50,
                .y = 50
            },
        .diameter = 20
        }
    };

    obstacle_t b = {
        .type = obstacle_type_circle,
        .data.circle = {
            .coordinates = {
                .x = 60,
                .y = 50
            },
        .diameter = 20
        }
    };

    TEST_ASSERT_EQUAL_MESSAGE(1, obstacle_are_they_colliding(&a, &b), "A and B must collide");
    b.data.circle.coordinates.x = 100;
    TEST_ASSERT_EQUAL_MESSAGE(0, obstacle_are_they_colliding(&a, &b), "A and B must not collide");
}

void test_collision_rectangles(void)
{
    obstacle_t a = {
        .type = obstacle_type_rectangle,
        .data.rectangle = {
            .coordinates = {
                .x = 50,
                .y = 50
            },
            .height = 20,
            .width = 20
        }
    };

    obstacle_t b = {
        .type = obstacle_type_rectangle,
        .data.rectangle = {
            .coordinates = {
                .x = 60,
                .y = 60
            },
            .height = 20,
            .width = 20
        }
    };

    TEST_ASSERT_EQUAL_MESSAGE(1, obstacle_are_they_colliding(&a, &b), "A and B must collide, 255 not supported");
    b.data.rectangle.coordinates.x = 100;
    TEST_ASSERT_EQUAL_MESSAGE(0, obstacle_are_they_colliding(&a, &b), "A and B must not collide");
}

void test_collision_rectangles_and_circles(void)
{
    obstacle_t a = {
        .type = obstacle_type_rectangle,
        .data.rectangle = {
            .coordinates = {
                .x = 50,
                .y = 50
            },
            .height = 20,
            .width = 20
        }
    };

    obstacle_t b = {
        .type = obstacle_type_circle,
        .data.circle = {
            .coordinates = {
                .x = 60,
                .y = 50
            },
        .diameter = 20
        }
    };

    TEST_ASSERT_EQUAL_MESSAGE(1, obstacle_are_they_colliding(&a, &b), "A and B must collide");
    TEST_ASSERT_EQUAL_MESSAGE(1, obstacle_are_they_colliding(&b, &a), "A and B must collide");
    b.data.circle.coordinates.x = 100;
    TEST_ASSERT_EQUAL_MESSAGE(0, obstacle_are_they_colliding(&a, &b), "A and B must not collide");
    TEST_ASSERT_EQUAL_MESSAGE(0, obstacle_are_they_colliding(&b, &a), "A and B must not collide");
}

void test_object_holder(){
    obstacle_holder_t holder = {0};
    obstacle_t ob = {
        .type = obstacle_type_circle,
        .data.circle = {
            .coordinates = {
                .x = 60,
                .y = 50
            },
        .diameter = 20
        }
    };

    TEST_ASSERT_EQUAL_MESSAGE(0,obstacle_holder_push(&holder, &ob), "Push is not valid");
    TEST_ASSERT_EQUAL_MESSAGE(0,obstacle_holder_push(&holder, &ob), "Push is not valid");
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE(&ob, &holder.obstacles[1], sizeof(obstacle_t), "Store object is not valid");
    
}


void test_object_holder_relative_storing(){
    lidar_message_t message = {0};
    message.start_angle = 0;
    message.end_angle = 80;
    message.points[0].quality = 1;
    message.points[0].distance = 100;
    obstacle_holder_t holder = {0};
    robot_t robot = {0};
    obstacle_t expected_result = {
        .type = obstacle_type_circle,
        .data.circle = {
            .coordinates = {
                .x = 0,
                .y = 100
            },
            .diameter = 4
        }
    };
    obstacle_t *result;

    TEST_ASSERT_EQUAL_MESSAGE(0, 
    relative_obstacle_storing_lidar_points_relative_to_robot(
        &holder, &message, &robot, 0
    ), "Error when storing lidar message");
    TEST_ASSERT_EQUAL_MESSAGE(0, obstacle_holder_get(&holder, &result), "Error when getting obstacle");
    TEST_ASSERT_EQUAL(expected_result.type, result->type);
    TEST_ASSERT_EQUAL(expected_result.data.circle.diameter, result->data.circle.diameter);
    TEST_ASSERT_EQUAL(expected_result.data.circle.coordinates.y, result->data.circle.coordinates.y);
    TEST_ASSERT_EQUAL(expected_result.data.circle.coordinates.x, result->data.circle.coordinates.x);
}

void test_object_collision_with_seg_and_circle(){

    coordinates_t start = {
        .x = 10,
        .y = 10,
    };
    coordinates_t end = {
        .x = 10,
        .y = 1000,
    };

    obstacle_t ob = {
        .type = obstacle_type_circle,
        .data.circle.coordinates = {
            .x = 10,
            .y = 500
        },
        .data.circle.diameter = 20,
    };

    uint16_t robot_diameter = 20;
    coordinates_t intersection_pt = {0};
    int rcode = obstacle_get_point_of_collision_with_segment(&start, &end, &ob, &robot_diameter, &intersection_pt);
    TEST_ASSERT_EQUAL_MESSAGE(1, rcode, "Intersection not found");

    ob.data.circle.coordinates.x = end.x + (ob.data.circle.diameter + robot_diameter)/2 + 1; 
    // FIXME: current method is aproximate even working with ob.data.circle.coordinates.x -= 1
    rcode = obstacle_get_point_of_collision_with_segment(&start, &end, &ob, &robot_diameter, &intersection_pt);
    TEST_ASSERT_EQUAL_MESSAGE(0, rcode, "Intersection found");
}

void test_object_collision_with_seg_and_rectangle(){

    coordinates_t start = {
        .x = 10,
        .y = 10,
    };
    coordinates_t end = {
        .x = 10,
        .y = 1000,
    };

    obstacle_t ob = {
        .type = obstacle_type_rectangle,
        .data.rectangle.coordinates = {
            .x = 10,
            .y = 500
        },
        .data.rectangle.width = 10,
        .data.rectangle.height = 10,
    };

    uint16_t robot_diameter = 20;
    coordinates_t intersection_pt = {0};
    int rcode = obstacle_get_point_of_collision_with_segment(&start, &end, &ob, &robot_diameter, &intersection_pt);
    TEST_ASSERT_EQUAL_MESSAGE(1, rcode, "Intersection not found");

    ob.data.rectangle.coordinates.x = end.x + (ob.data.rectangle.width + robot_diameter)/2 + 1; 
    // FIXME: current method is aproximate even working with ob.data.circle.coordinates.x -= 1
    rcode = obstacle_get_point_of_collision_with_segment(&start, &end, &ob, &robot_diameter, &intersection_pt);
    TEST_ASSERT_EQUAL_MESSAGE(0, rcode, "Intersection found");
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_collision_circles);
    // RUN_TEST(test_collision_rectangles);
    RUN_TEST(test_collision_rectangles_and_circles);
    RUN_TEST(test_object_holder);
    RUN_TEST(test_object_holder_relative_storing);
    RUN_TEST(test_object_collision_with_seg_and_circle);
    RUN_TEST(test_object_collision_with_seg_and_rectangle);
    return UNITY_END();
}