#include "border_recoil.h"

#include "algo.h"
#include "obstacle_avoidance.h"

#include "reflectance.h"

// constants
#define BORDER_RECOIL_COEFFICIENT_DEC_PER_TICK (0.015)

// global variables
border_recoil_state_type border_recoil_state;
float border_recoil_coefficient;

// local variables
static const char *TAG = "algo-border-recoil";

// function declarations
static void border_recoil_run_state();
static void border_recoil_output();
static void border_recoil_start();
static void border_recoil_clear_reflectance_request();

// function definitions

void border_recoil_init()
{
}

void border_recoil_calculate()
{
    border_recoil_run_state();
    border_recoil_output();

    // if (border_recoil_state != BORDER_RECOIL_NONE)
    // {
    //     ESP_LOGE(TAG, "Border recoil state %d", border_recoil_state);
    //     algo_request_stop();
    // }
}

void border_recoil_run_state()
{
    switch (border_recoil_state)
    {
    case BORDER_RECOIL_NONE:
        if (reflectance_request_avoidance.left)
        {
            /**
             * @brief Left sensor was triggered, recoil to the right.
             */
            ESP_LOGW(TAG, "Left reflectance sensor triggered, recoil to the right");
            border_recoil_state = BORDER_RECOIL_DIRECTION_RIGHT;
            border_recoil_start();
        }
        else if (reflectance_request_avoidance.right)
        {
            /**
             * @brief Right sensor was triggered, recoil to the left.
             * @todo Uncomment when right sensor will work.
             */
            ESP_LOGW(TAG, "Right reflectance sensor triggered, recoil to the left");
            border_recoil_state = BORDER_RECOIL_DIRECTION_LEFT;
            border_recoil_start();
        }
        break;
    case BORDER_RECOIL_DIRECTION_LEFT:
    case BORDER_RECOIL_DIRECTION_RIGHT:
        border_recoil_coefficient -= BORDER_RECOIL_COEFFICIENT_DEC_PER_TICK;
        if (border_recoil_coefficient <= 0)
        {
            ESP_LOGW(TAG, "Stopping recoiling from the border");
            border_recoil_clear_reflectance_request();
            border_recoil_reset();
        }
    }
}

void border_recoil_output()
{
    switch (algo_obstacle_avoidance_state)
    {
    case OA_BEHAVIOUR_FOLLOW_WALL_CLOCKWISE:
        if (border_recoil_state == BORDER_RECOIL_DIRECTION_RIGHT)
        {
            obstacle_avoidance_force_ccw();
        }
        break;
    case OA_BEHAVIOUR_FOLLOW_WALL_COUNTERCLOCKWISE:
        if (border_recoil_state == BORDER_RECOIL_DIRECTION_LEFT)
        {
            obstacle_avoidance_force_cw();
        }
    default:
        break;
    }
}

void border_recoil_clear_reflectance_request()
{
    /**
     * @brief These writes are not multi-core safe.
     * However, as it is only a flag, it's not a problem to the algorithm.
     */
    switch (border_recoil_state)
    {
    case BORDER_RECOIL_DIRECTION_LEFT:
        reflectance_reset();
        break;
    case BORDER_RECOIL_DIRECTION_RIGHT:
        reflectance_reset();
        break;
    default:
        break;
    }
}

void border_recoil_start()
{
    border_recoil_coefficient = 1.0;
}

void border_recoil_reset()
{
    border_recoil_state = BORDER_RECOIL_NONE;
    border_recoil_coefficient = 0.0;
}