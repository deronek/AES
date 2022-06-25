#include "final_heading.h"

#include "obstacle_avoidance.h"
#include "goal_heading.h"

#include <math.h>

// global variables
float algo_final_heading;
final_heading_behaviour_state_type algo_final_heading_behaviour_state = BEHAVIOUR_DRIVE_TO_GOAL;

// local variables
static const char *TAG = "algo-final-heading";
float distance_to_goal;

// function declarations

// inline function definitions

// function definitions

void final_heading_init()
{
    final_heading_reset();
}

void final_heading_calculate()
{
    switch (algo_obstacle_avoidance_state)
    {
    case OA_BEHAVIOUR_NONE:
        algo_final_heading_behaviour_state = BEHAVIOUR_DRIVE_TO_GOAL;
        algo_final_heading = algo_goal_heading;
        break;
    case OA_BEHAVIOUR_FOLLOW_WALL_CLOCKWISE:
    case OA_BEHAVIOUR_FOLLOW_WALL_COUNTERCLOCKWISE:
        algo_final_heading_behaviour_state = BEHAVIOUR_FOLLOW_THE_WALL;
        algo_final_heading = algo_follow_wall_angle;
        break;
    case OA_BEHAVIOUR_AVOID_OBSTACLE:
        algo_final_heading_behaviour_state = BEHAVIOUR_AVOID_OBSTACLE;
        algo_final_heading = algo_avoid_obstacle_angle;
    }
}

void final_heading_reset()
{
    algo_final_heading_behaviour_state = BEHAVIOUR_DRIVE_TO_GOAL;
}