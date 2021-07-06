#include <unity.h>
#include "strategy.h"

uint8_t action_done;

uint32_t return_zero_cost(const goal_t *this)
{
    return 0;
}

uint32_t return_one_reward(const goal_t *this)
{
    return 1;
}

uint8_t do_action(const goal_t *this)
{
    action_done = 1;
    return 0; 
}

uint8_t do_action_2(const goal_t *this)
{
    action_done = 2;
    return 0; 
}

void setUp(void)
{
    action_done = 0;
}

void tearDown(void)
{
    // clean stuff up here
}

void test_init_obj_ok(void)
{
    STRATEGY_BUILD_INIT(obj)
    STRATEGY_BUILD_ADD(return_zero_cost, do_action, return_one_reward, NULL, status_ready)
    STRATEGY_BUILD_END(obj);
    TEST_ASSERT_EQUAL_MESSAGE(1, obj.nb_goals, "Must have one goal");
    goal_t * gl = strategy_get_best_goal(&obj);
    TEST_ASSERT_NOT_NULL(gl);
}

void test_strategy_run(void)
{
    STRATEGY_BUILD_INIT(obj)
    STRATEGY_BUILD_ADD(return_zero_cost, do_action, return_one_reward, NULL, status_ready)
    STRATEGY_BUILD_END(obj);
    TEST_ASSERT_EQUAL_MESSAGE(1, obj.nb_goals, "Must have one goal");
    uint8_t err = strategy_run(&obj);
    TEST_ASSERT_NULL_MESSAGE(err, "strategy_run must not return an error");
    TEST_ASSERT_EQUAL_MESSAGE(1, action_done, "do_action must have been ran");
}

void test_strategy_run_multiple(void)
{
    STRATEGY_BUILD_INIT(obj)
    STRATEGY_BUILD_ADD(return_zero_cost, do_action, return_one_reward, NULL, status_ready)
    STRATEGY_BUILD_ADD(return_zero_cost, do_action_2, return_one_reward, NULL, status_ready)
    STRATEGY_BUILD_END(obj);
    TEST_ASSERT_EQUAL_MESSAGE(2, obj.nb_goals, "Must have 2 goal");
    uint8_t err = strategy_run(&obj);
    TEST_ASSERT_EQUAL_MESSAGE(0, err, "strategy_run must not return an error");
    TEST_ASSERT_EQUAL_MESSAGE(2, action_done, "do_action must have been ran");
}


int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_obj_ok);
    RUN_TEST(test_strategy_run);
    RUN_TEST(test_strategy_run_multiple);
    return UNITY_END();
}