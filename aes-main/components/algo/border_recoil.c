#include "border_recoil.h"

#include "obstacle_avoidance.h"

#include "reflectance.h"

// constants
#define BORDER_RECOIL_COEFFICIENT_DEC_PER_TICK (0.05)

// local variables
border_recoil_state_type border_recoil_state;
float border_recoil_coefficient;

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
            border_recoil_state = BORDER_RECOIL_DIRECTION_RIGHT;
            border_recoil_start();
        }
        else if (reflectance_request_avoidance.right)
        {
            /**
             * @brief Right sensor was triggered, recoil to the left.
             */
            border_recoil_state = BORDER_RECOIL_DIRECTION_LEFT;
            border_recoil_start();
        }
        break;
    case BORDER_RECOIL_DIRECTION_LEFT:
    case BORDER_RECOIL_DIRECTION_RIGHT:
        border_recoil_coefficient -= BORDER_RECOIL_COEFFICIENT_DEC_PER_TICK;
        if (border_recoil_coefficient < 0)
        {
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
        if (border_recoil_state == BORDER_RECOIL_DIRECTION_LEFT)
        {
            obstacle_avoidance_force_ccw();
        }
        break;
    case OA_BEHAVIOUR_FOLLOW_WALL_COUNTERCLOCKWISE:
        if (border_recoil_state == BORDER_RECOIL_DIRECTION_RIGHT)
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
        reflectance_request_avoidance.right = false;
        break;
    case BORDER_RECOIL_DIRECTION_RIGHT:
        reflectance_request_avoidance.left = false;
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