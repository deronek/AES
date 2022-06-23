#include "obstacle_avoidance.h"

#include "data_receive.h"

#include "hc_sr04.h"
#include "heading_imu.h"
#include "desired_heading.h"

#include "esp_log.h"
#include "esp_err.h"

#include <stdlib.h>

#include <math.h>

// constants

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)

#define DISTANCE_SCALING_FACTOR_UNITY 10000
#define DANGER_LEVEL_DISTANCE_THRESHOLD 800000

#define DANGER_LEVEL_SAFE_THRESHOLD (3.0)
#define DANGER_LEVEL_DECREMENT_PER_TICK (0.2)

#define SECTOR_NOT_FOUND (UINT8_MAX)

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

/**
 * @brief HC-SR04 sensor regions - start angles
 * Sector are numbered from left to right, from 0 to 7.
 */
// static const float hc_sr04_sensor_regions[NUMBER_OF_HC_SR04_SENSORS] = {
//     90, 67.5, 45, 22.5, 0, -22.5, -45, -67.5};
#define SECTOR_ANGLE (22.5 * DEG_TO_RAD)
#define SECTOR_ANGLE_HALF (SECTOR_ANGLE / 2.0)

static const float hc_sr04_sensor_regions[NUMBER_OF_HC_SR04_SENSORS] = {
    90.0 * DEG_TO_RAD,
    67.5 * DEG_TO_RAD,
    45 * DEG_TO_RAD,
    22.5 * DEG_TO_RAD,
    0 * DEG_TO_RAD,
    -22.5 * DEG_TO_RAD,
    -45 * DEG_TO_RAD,
    -67.5 * DEG_TO_RAD};

// static const float hc_sr04_sensor_regions_centers[NUMBER_OF_HC_SR04_SENSORS] = {
//     90.0 * DEG_TO_RAD + SECTOR_ANGLE_HALF,
//     67.5 * DEG_TO_RAD + SECTOR_ANGLE_HALF,
//     45 * DEG_TO_RAD + SECTOR_ANGLE_HALF,
//     22.5 * DEG_TO_RAD + SECTOR_ANGLE_HALF,
//     0 * DEG_TO_RAD + SECTOR_ANGLE_HALF,
//     -22.5 * DEG_TO_RAD + SECTOR_ANGLE_HALF,
//     -45 * DEG_TO_RAD + SECTOR_ANGLE_HALF,
//     -67.5 * DEG_TO_RAD + SECTOR_ANGLE_HALF};

#define REGION_ANGLE (M_PI / NUMBER_OF_HC_SR04_SENSORS)

// global variables
float algo_obstacle_avoidance_steering_angle;
uint8_t algo_obstacle_avoidance_heading_sector;
uint8_t algo_obstacle_avoidance_danger_level_in_heading;
float algo_follow_wall_angle;

// local variables
static const char *TAG = "algo-obstacle-avoidance";
float obstacle_avoidance_angles[NUMBER_OF_HC_SR04_SENSORS];
float danger_intensities[NUMBER_OF_HC_SR04_SENSORS];
int most_dangerous_sector = -1;
float biggest_danger_level = -1;
float danger_levels[NUMBER_OF_HC_SR04_SENSORS];

// local function declarations
static float fuzzy_input_near(uint32_t distance);
static float fuzzy_input_middle(uint32_t distance);
static float fuzzy_input_far(uint32_t distance);
static void calculate_danger_intensities();
static void calculate_danger_levels();
static int calculate_sector_from_heading(float heading);
static void check_left_sector(int sector_to_check, int *safe_sector);
static void check_right_sector(int sector_to_check, int *safe_sector);
void calculate_obstacle_avoidance_angle();

// inline function definitions
inline static bool is_correct_sector(int sector)
{
    if (sector >= 0 && sector < NUMBER_OF_HC_SR04_SENSORS)
    {
        return true;
    }
    else
    {
        return false;
    }
}

inline static bool is_sector_safe(int sector)
{
    if (is_correct_sector(sector) && (danger_levels[sector] < DANGER_LEVEL_SAFE_THRESHOLD))
    {
        return true;
    }
    else
    {
        return false;
    }
}

// function definitions

void obstacle_avoidance_init()
{
}

void obstacle_avoidance_calculate()
{
    // /**
    //  * @brief Danger intensity defines how risky is to head
    //  * to the direction pointed by the sensor.
    //  */
    // calculate_danger_intensities();

    /**
     * @brief Boundary following algorithm:
     * - get obstacle avoidance angle from each obstacle (sector)
     * - choose clockwise or counter-clockwise direction (maybe which is more
     * in the way of the goal)
     * - calculate "follow the wall angle"
     * - this angle will be blended in final heading module
     */
    /**
     * @brief Danger level defines how much corrective action
     * the robot has to take to avoid collision.
     */
    calculate_danger_levels();
    if (biggest_danger_level < DANGER_LEVEL_SAFE_THRESHOLD)
    {
        /**
         * @todo Do not avoid the obstacle, area is safe.
         */
        algo_follow_wall_angle = INFINITY;
        ESP_LOGI(TAG, "Area is safe, driving to desired heading");
        return;
    }

    /**
     * @brief Calculate obstacle avoidance angle to the nearest obstacle
     * (most dangerous sector).
     */
    calculate_obstacle_avoidance_angle();
    ESP_LOGI(TAG, "Following wall at angle %.2f, obstacle at angle %.2f", algo_follow_wall_angle * RAD_TO_DEG, hc_sr04_sensor_regions[most_dangerous_sector] * RAD_TO_DEG);
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

void calculate_obstacle_avoidance_angle()
{
    float angle_to_obstacle = algo_current_heading + hc_sr04_sensor_regions[most_dangerous_sector];
    float cw = angle_to_obstacle + M_PI;
    float ccw = angle_to_obstacle - M_PI;

    /**
     * @brief If clockwise direction is closer to the desired heading, choose it.
     * Otherwise, choose the counter-clockwise direction.
     */
    if (fabsf((algo_desired_heading - cw)) < fabsf((algo_desired_heading - ccw)))
    {
        algo_follow_wall_angle = cw;
    }
    else
    {
        algo_follow_wall_angle = ccw;
    }
}

void check_left_sector(int sector_to_check, int *safe_sector)
{
    int left_sector = sector_to_check;
    if (is_sector_safe(left_sector))
    {
        *safe_sector = left_sector;
        return;
    }
}

void check_right_sector(int sector_to_check, int *safe_sector)
{
    int right_sector = sector_to_check;
    if (is_sector_safe(right_sector))
    {
        *safe_sector = right_sector;
        return;
    }
}

int calculate_sector_from_heading(float heading)
{
    if (heading >= hc_sr04_sensor_regions[1])
    {
        return 0;
    }
    if (heading <= hc_sr04_sensor_regions[NUMBER_OF_HC_SR04_SENSORS - 1])
    {
        return NUMBER_OF_HC_SR04_SENSORS - 1;
    }

    for (int i = 1; i < NUMBER_OF_HC_SR04_SENSORS - 1; ++i)
    {
        if (heading <= hc_sr04_sensor_regions[i] && heading >= hc_sr04_sensor_regions[i + 1])
        {
            return i;
        }
    }
    ESP_LOGE(TAG, "calculate_sector_from_heading error");
    return -1;
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
    most_dangerous_sector = -1;
    biggest_danger_level = -1;
    for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
    {
        float new_danger_level;
        uint32_t distance = algo_hc_sr04_data.distance[i];

        // measurement error, put maximum danger level
        if (distance == 0)
        {
            new_danger_level = DANGER_LEVEL_MAX;
        }

        // very far away, minimum danger level
        else if (distance > DANGER_LEVEL_DISTANCE_THRESHOLD)
        {
            new_danger_level = DANGER_LEVEL_MIN;
        }

        else
        {
            new_danger_level = min(DANGER_LEVEL_MAX, (float)DANGER_LEVEL_DISTANCE_THRESHOLD / distance);
        }

        /**
         * @brief If new danger level is higher than before, set it.
         *
         * If danger level difference is smaller than DANGER_LEVEL_DECREMENT_PER_TICK,
         * also set new danger level.
         *
         * If danger level difference is larger than DANGER_LEVEL_DECREMENT_PER_TICK,
         * decrement danger level by DANGER_LEVEL_DECREMENT_PER_TICK.
         */
        if (new_danger_level > danger_levels[i] - DANGER_LEVEL_DECREMENT_PER_TICK)
        {
            danger_levels[i] = new_danger_level;
        }
        {
            danger_levels[i] -= DANGER_LEVEL_DECREMENT_PER_TICK;
        }

        /**
         * @brief Check if this is new most dangerous sector.
         */
        if (new_danger_level > biggest_danger_level)
        {
            biggest_danger_level = new_danger_level;
            most_dangerous_sector = i;
        }
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