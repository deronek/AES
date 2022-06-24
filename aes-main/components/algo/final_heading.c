#include "final_heading.h"

#include "obstacle_avoidance.h"
#include "goal_heading.h"

#include <math.h>

// global variables
float algo_final_heading;
final_heading_behaviour_state_type final_heading_behaviour_state = BEHAVIOUR_DRIVE_TO_GOAL;

// local variables
static const char *TAG = "algo-final-heading";
float distance_to_goal;

// function declarations

static void final_heading_run_state();
static void final_heading_output();

// inline function definitions

// function definitions

void final_heading_init()
{
    final_heading_reset();
}

void final_heading_calculate()
{
    /**
     * @brief Run state machine transition, then calculate final heading output.
     */
    final_heading_run_state();
    ESP_LOGI(TAG, "State: %d", final_heading_behaviour_state);
    final_heading_output();
}

void final_heading_run_state()
{
    switch (final_heading_behaviour_state)
    {
    case BEHAVIOUR_DRIVE_TO_GOAL:
        /**
         * @brief Obstacle is nearby, transition into following the wall.
         */
        if (algo_obstacle_avoidance_request_follow_wall)
        {
            final_heading_behaviour_state = BEHAVIOUR_FOLLOW_THE_WALL;
            distance_to_goal = goal_heading_distance_to_goal();
        }
        break;
    case BEHAVIOUR_FOLLOW_THE_WALL:
        /**
         * @brief Check condition for transitioning back to driving to goal:
         * - progress made to goal (lower distance than at the start),
         * - goal heading and obstacle avoidance angle is less than M_PI / 2.
         */
        if ((goal_heading_distance_to_goal() < distance_to_goal) && (fabsf(algo_goal_heading - algo_obstacle_avoidance_angle) < (M_PI / 2)))
        {
            final_heading_behaviour_state = BEHAVIOUR_DRIVE_TO_GOAL;
        }
    }
}

void final_heading_output()
{
    switch (final_heading_behaviour_state)
    {
    case BEHAVIOUR_DRIVE_TO_GOAL:
        algo_final_heading = algo_goal_heading;
        break;
    case BEHAVIOUR_FOLLOW_THE_WALL:
        algo_final_heading = algo_follow_wall_angle;
        break;
    }
}

void final_heading_reset()
{
    final_heading_behaviour_state = BEHAVIOUR_DRIVE_TO_GOAL;
}
