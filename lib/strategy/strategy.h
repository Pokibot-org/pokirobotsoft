#ifndef STRATEGY_H
#define STRATEGY_H
#include "stdint.h"

struct goal;

typedef enum status {
    status_unused,
    status_ready,
    status_completed,
    status_always_ready
} status_t;


typedef uint8_t (*action_fun_t)(const struct goal *);
typedef uint32_t (*score_fun_t)(const struct goal *);


typedef struct goal
{
    score_fun_t score_callback;
    action_fun_t action_callback;
    void * user_data;
    status_t status;
} goal_t;

typedef struct strategy
{
    goal_t * goals;
    uint16_t nb_goals;
} strategy_t;

/*
#define STRATEGY_INIT(_obj_name, _nb_goals) \
    do { \
        goal_t goal_array__obj_name[_nb_goals] = {0}; \
        strategy_t _obj_name = {goal_array__obj_name, _nb_goals}; } \
    while(0)
*/

#define STRATEGY_BUILD_INIT(_obj_name) goal_t goal_array__obj_name[] = {

#define STRATEGY_BUILD_ADD(score_clbk, action_clbk, user_data, status) \
    {score_clbk, action_clbk, user_data, status},

#define STRATEGY_BUILD_END(_obj_name) \
    }; \
    strategy_t _obj_name = {(goal_array__obj_name), sizeof((goal_array__obj_name))/sizeof((goal_array__obj_name)[0])};


goal_t * strategy_get_best_goal(const strategy_t *obj);
uint8_t strategy_run(const strategy_t *obj);

#endif