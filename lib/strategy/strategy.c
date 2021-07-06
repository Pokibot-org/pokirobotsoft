#include "strategy.h"
#include "stdlib.h"


goal_t * strategy_get_best_goal(const strategy_t *obj)
{
    goal_t * best_goal = NULL;
    for (uint32_t i = 0; i < obj->nb_goals; i++)
    {
        if (obj->goals[i].status != status_completed && obj->goals[i].status != status_unused)
        {
            best_goal = &obj->goals[i];
            break;
        }
    }
    return best_goal;
}

uint8_t strategy_run(const strategy_t *obj){
    while (1)
    {
        goal_t *current_goal = strategy_get_best_goal(obj);
        if (current_goal == NULL)
        {
            break;
        }

        if (current_goal->action_callback == NULL)
        {
            return 1;
        }
        current_goal->action_callback(current_goal);
        if (current_goal->status == status_ready)
        {
            current_goal->status = status_completed;
        }
    }
    return 0;
}