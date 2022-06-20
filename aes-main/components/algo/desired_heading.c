#include "desired_heading.h"

#include "algo.h"
#include "data_receive.h"
#include "position.h"

// global variables
float algo_desired_heading;

// local variables

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
static float xf = 2.0;
static float yf = 6.6;

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
    float a = algo_position.x - x_hat;
    float b = algo_position.y - y_hat;

    algo_desired_heading = atan2f(b, a);
}