#include "obstacle_avoidance.h"

#include "data_receive.h"

#include "hc_sr04.h"

#include "esp_log.h"
#include "esp_err.h"

#include <stdlib.h>

// constants
#define DISTANCE_SCALING_FACTOR_UNITY 10000

// #define DANGER_LEVELS_NUMBER 7
#define DANGER_LEVEL_MAX 5
#define DANGER_LEVEL_DISTANCE_THRESHOLD 60000

// fuzzy input boundaries
#define FUZZY_INPUT_BOUNDARY_NEAR_LOW 10000
#define FUZZY_INPUT_BOUNDARY_NEAR_HIGH 20000
#define FUZZY_INPUT_BOUNDARY_FAR_LOW 40000
#define FUZZY_INPUT_BOUNDARY_FAR_HIGH 60000

// fuzzy output weight factors
#define FUZZY_OUTPUT_NEAR 1.0
#define FUZZY_OUTPUT_MIDDLE 0.5
#define FUZZY_OUTPUT_FAR 0.0

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

// local variables
float *danger_intensities;
float *danger_levels;

// global variables

// local function declarations
static float fuzzy_input_near(uint32_t distance);
static float fuzzy_input_middle(uint32_t distance);
static float fuzzy_input_far(uint32_t distance);
static void calculate_danger_intensities();
static void calculate_danger_levels();

// function definitions

void obstacle_avoidance_init()
{
    danger_intensities = malloc(NUMBER_OF_HC_SR04_SENSORS * sizeof(danger_intensities));
    if (danger_intensities == NULL)
    {
        abort();
    }

    danger_levels = malloc(NUMBER_OF_HC_SR04_SENSORS * sizeof(danger_intensities));
    if (danger_levels == NULL)
    {
        abort();
    }
}

void obstacle_avoidance_calculate()
{
    /**
     * @brief Danger intensity defines how risky is to head
     * to the direction pointed by the sensor.
     */
    calculate_danger_intensities();

    /**
     * @brief Danger level defines how much corrective action
     * the robot has to take to avoid collision.
     */
    calculate_danger_levels();

    /**
     * @brief IEEE article approach
     * - check if it is safe to drive to wanted heading
     * - if yes, drive to it (with small adjustement according to danger intensity
     * of adjacent cells)
     * - if not, try adjacent heading. First try the one closer to current vehicle
     * trajectory.
     */

    /**
     * @brief Follow the gap approach
     * - find maximum gap angle
     * - if we are close to the obstacle, we will drive to it
     * - if not, we just follow planning heading angle
     */
}

void calculate_danger_intensities()
{
    for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
    {
        uint32_t distance = algo_hc_sr04_data.distance[i];
        float near = fuzzy_input_near(distance);
        float middle = fuzzy_input_middle(distance);
        float far = fuzzy_input_far(distance);

        float nominator = near * FUZZY_OUTPUT_NEAR +
                          middle * FUZZY_OUTPUT_MIDDLE +
                          far * FUZZY_OUTPUT_FAR;

        float denominator = near + middle + far;

        danger_intensities[i] = nominator / denominator;
    }
}

void calculate_danger_levels(uint32_t distance)
{
    for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
    {
        uint32_t distance = algo_hc_sr04_data.distance[i];
        if (distance > DANGER_LEVEL_DISTANCE_THRESHOLD)
        {
            return 0;
        }

        danger_levels[i] = min(DANGER_LEVEL_MAX, DANGER_LEVEL_DISTANCE_THRESHOLD / distance);
    }
}

/**
 * @brief Fuzzifier transforming distance in micrometers to
 * three fuzzy states - near, middle, far.
 * @param distance in micrometers
 * @return float
 */

float fuzzy_input_near(uint32_t distance)
{
    if (distance < FUZZY_INPUT_BOUNDARY_NEAR_LOW)
    {
        return 1;
    }

    if (distance >= FUZZY_INPUT_BOUNDARY_NEAR_LOW &&
        distance < FUZZY_INPUT_BOUNDARY_NEAR_HIGH)
    {
        return (float)(FUZZY_INPUT_BOUNDARY_NEAR_HIGH - distance) /
               (FUZZY_INPUT_BOUNDARY_NEAR_HIGH - FUZZY_INPUT_BOUNDARY_NEAR_LOW);
    }

    return 0;
}

float fuzzy_input_middle(uint32_t distance)
{
    if (distance < FUZZY_INPUT_BOUNDARY_NEAR_LOW)
    {
        return 0;
    }

    if (distance >= FUZZY_INPUT_BOUNDARY_NEAR_LOW &&
        distance < FUZZY_INPUT_BOUNDARY_NEAR_HIGH)
    {
        return (float)(distance - FUZZY_INPUT_BOUNDARY_NEAR_LOW) /
               (FUZZY_INPUT_BOUNDARY_NEAR_HIGH - FUZZY_INPUT_BOUNDARY_NEAR_LOW);
    }

    if (distance >= FUZZY_INPUT_BOUNDARY_NEAR_HIGH &&
        distance < FUZZY_INPUT_BOUNDARY_FAR_LOW)
    {
        return 1;
    }

    if (distance >= FUZZY_INPUT_BOUNDARY_FAR_LOW &&
        distance < FUZZY_INPUT_BOUNDARY_FAR_HIGH)
    {
        return (float)(FUZZY_INPUT_BOUNDARY_FAR_HIGH - distance) /
               (FUZZY_INPUT_BOUNDARY_FAR_HIGH - FUZZY_INPUT_BOUNDARY_FAR_LOW);
    }

    return 0;
}

float fuzzy_input_far(uint32_t distance)
{
    if (distance < FUZZY_INPUT_BOUNDARY_FAR_LOW)
    {
        return 0;
    }

    if (distance >= FUZZY_INPUT_BOUNDARY_FAR_LOW &&
        distance < FUZZY_INPUT_BOUNDARY_FAR_HIGH)
    {
        return (float)(distance - FUZZY_INPUT_BOUNDARY_FAR_LOW) /
               (FUZZY_INPUT_BOUNDARY_FAR_HIGH - FUZZY_INPUT_BOUNDARY_FAR_LOW);
    }

    return 1;
}