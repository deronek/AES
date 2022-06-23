#include "desired_heading.h"

#include "algo.h"
#include "data_receive.h"
#include "position.h"

// constants

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)

// global variables
float algo_desired_heading;

// local variables
static const char *TAG = "algo-desired-heading";

/**
 * @brief Refactor all of below data as parameters (maybe passed via BLE).
 * Assume the (0, 0) point is the bottom right point and coordinates
 * can be only positive.
 */

/**
 * @brief (xs, ys) - coordinates of the start point of the vehicle.
 */
static float xs = 0.0;
static float ys = 0.0;

/**
 * @brief (xf, yf) - coordinates of the finish point.
 */
static float xf = 3.96;
static float yf = 2.0;

/**
 * @brief (x_hat, y_hat) - length and with of the play area.
 */
#define INCH_TO_CM
static float x_hat = 3.96;
static float y_hat = 6.6;

// function definitions
void desired_heading_init()
{
}

void desired_heading_calculate()
{
    float a = xf - algo_position.x;
    float b = yf - algo_position.y;

    algo_desired_heading = atan2f(a, b);
    ESP_LOGI(TAG, "Desired heading: %.2f", RAD_TO_DEG * algo_desired_heading);
}