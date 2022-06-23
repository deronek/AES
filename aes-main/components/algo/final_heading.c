#include "final_heading.h"

#include "obstacle_avoidance.h"
#include "desired_heading.h"

// global variables
float algo_final_heading;

void final_heading_init()
{
}

void final_heading_calculate()
{
    /**
     * @brief Calculate final heading angle.
     * We use waged calculation based on danger level in heading direction.
     * If it's high, focus on avoiding the obstacle.
     * If it's low, focus on driving to the desired heading.
     */
    float avoidance_weight = algo_obstacle_avoidance_danger_level_in_heading / DANGER_LEVEL_MAX;

    algo_final_heading = (avoidance_weight * algo_obstacle_avoidance_steering_angle + algo_desired_heading) / (avoidance_weight + 1);
}