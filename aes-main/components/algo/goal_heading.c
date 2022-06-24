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

/**
 * @brief Refactor all of below data as parameters (maybe passed via BLE).
 * Assume the (0, 0) point is the bottom right point and coordinates
 * can be only positive.
 */

/**
 * @brief (xs, ys) - coordinates of the start point of the vehicle.
 */
static float xs = 0.75;
static float ys = 0;

/**
 * @brief (xf, yf) - coordinates of the finish point.
 */
static float xf = 3.96;
static float yf = 3.66;

// static float xf = 0.75;
// static float yf = 50;

/**
 * @brief (x_hat, y_hat) - length and with of the play area.
 */

/**
 * @brief From WIKAMP:
 */
#define INCH_TO_CM
// static float x_hat = 3.96;
// static float y_hat = 6.6;

/**
 * @brief Measured:
 */
static float x_hat = 3.05;
static float y_hat = 6.1;

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
    float a = xf - algo_position.x;
    float b = yf - algo_position.y;

    return atan2f(a, b);
}

float goal_heading_distance_to_goal()
{
    float delta_x = algo_position.x - xf;
    float delta_y = algo_position.y - yf;
    float distance = sqrtf((delta_x * delta_x + delta_y * delta_y));

    return distance;
}