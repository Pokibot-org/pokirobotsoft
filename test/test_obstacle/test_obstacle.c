#include <unity.h>
#include "obstacle.h"
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

    TEST_ASSERT_EQUAL_MESSAGE(0,obstacle_holder_push(&holder, &ob), "Push is valid");
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_collision_circles);
    RUN_TEST(test_collision_rectangles);
    RUN_TEST(test_collision_rectangles_and_circles);
    RUN_TEST(test_object_holder);
    return UNITY_END();
}