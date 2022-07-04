#include "goal_heading.h"

#include "algo.h"
#include "data_receive.h"
#include "position.h"

// constants

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)

// global variables
float algo_goal_heading;

// local variables
static const char *TAG = "algo-goal-heading";

// function definitions
void goal_heading_init()
{
}

void goal_heading_calculate()
{
    algo_goal_heading = goal_heading_angle_to_goal();
    ESP_LOGI(TAG, "Desired heading: %.2f", RAD_TO_DEG * algo_goal_heading);
}

float goal_heading_angle_to_goal()
{
    float a = X_FINISH - algo_position.x;
    float b = Y_FINISH - algo_position.y;

    return atan2f(a, b);
}

float goal_heading_distance_to_goal()
{
    float delta_x = algo_position.x - X_FINISH;
    float delta_y = algo_position.y - Y_FINISH;
    float distance = sqrtf((delta_x * delta_x + delta_y * delta_y));

    return distance;
}