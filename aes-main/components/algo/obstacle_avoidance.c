#include "obstacle_avoidance.h"

#include "data_receive.h"

#include "hc_sr04.h"
#include "heading_imu.h"
#include "goal_heading.h"
#include "calc_utils.h"

#include "esp_log.h"
#include "esp_err.h"

#include <stdlib.h>

#include <math.h>

// constants

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)

// #define DISTANCE_SCALING_FACTOR_UNITY 10000
// #define DANGER_LEVEL_DISTANCE_THRESHOLD 800000

// #define DANGER_LEVEL_SAFE_THRESHOLD (3.0)
// #define DANGER_LEVEL_DECREMENT_PER_TICK (0.2)

#define DISTANCE_AVOID_THRESHOLD (200000)
#define DISTANCE_SAFE_THRESHOLD (600000)

// #define SECTOR_NOT_FOUND (UINT8_MAX)

// fuzzy input boundaries
// #define FUZZY_INPUT_BOUNDARY_NEAR_LOW 10000
// #define FUZZY_INPUT_BOUNDARY_NEAR_HIGH 20000
// #define FUZZY_INPUT_BOUNDARY_FAR_LOW 40000
// #define FUZZY_INPUT_BOUNDARY_FAR_HIGH 60000

// fuzzy output weight factors
// #define FUZZY_OUTPUT_NEAR 1.0
// #define FUZZY_OUTPUT_MIDDLE 0.5
// #define FUZZY_OUTPUT_FAR 0.0

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
// uint8_t algo_obstacle_avoidance_heading_sector;
// uint8_t algo_obstacle_avoidance_danger_level_in_heading;
float algo_follow_wall_angle;
float algo_avoid_obstacle_angle;
// bool algo_obstacle_avoidance_request_follow_wall;
obstacle_avoidance_state_type algo_obstacle_avoidance_state;

// local variables
static const char *TAG = "algo-obstacle-avoidance";
// float obstacle_avoidance_angles[NUMBER_OF_HC_SR04_SENSORS];
// float danger_intensities[NUMBER_OF_HC_SR04_SENSORS];
static int most_dangerous_sector = -1;
static int most_dangerous_sector_measurement = -1;
static int last_most_dangerous_sector = -1;
static int last_most_dangerous_sector_measurement = -1;

/**
 * @todo Fix type
 */
static uint32_t last_hc_sr04_measurement_number = 0;
static uint32_t follow_wall_ticks = 0;
static float distance_to_goal;
static float angle_to_obstacle;
static float follow_wall_cc_angle;
static float follow_wall_ccw_angle;
static float obstacle_avoidance_angle;

// local function declarations
// static float fuzzy_input_near(uint32_t distance);
// static float fuzzy_input_middle(uint32_t distance);
// static float fuzzy_input_far(uint32_t distance);
// static void calculate_danger_intensities();
// static void calculate_danger_levels();
// static int calculate_sector_from_heading(float heading);
// static void check_left_sector(int sector_to_check, int *safe_sector);
// static void check_right_sector(int sector_to_check, int *safe_sector);
static obstacle_avoidance_state_type calculate_follow_wall_behaviour();

static void calculate_obstacle_avoidance_angle();
static void calculate_most_dangerous_sector();
static void calculate_follow_wall_cc_angle();
static void calculate_follow_wall_ccw_angle();

static bool should_exit_follow_wall_behaviour();
static bool should_enter_follow_wall_behaviour();
static bool should_enter_avoid_obstacle_behaviour();
static bool should_exit_avoid_obstacle_behaviour();

static float get_angle_to_obstacle_from_sector(int sector);
static float get_avoidance_angle_from_angle(float angle_to_obstacle);

// inline function definitions
// inline static bool is_correct_sector(int sector)
// {
//     if (sector >= 0 && sector < NUMBER_OF_HC_SR04_SENSORS)
//     {
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }

// inline static bool is_sector_safe(int sector)
// {
//     if (is_correct_sector(sector) && (danger_levels[sector] < DANGER_LEVEL_SAFE_THRESHOLD))
//     {
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }

// function definitions

void obstacle_avoidance_init()
{
}

void obstacle_avoidance_calculate()
{
    ESP_LOGI(TAG, "last_measurement: %d, now: %d",
             last_hc_sr04_measurement_number,
             algo_hc_sr04_data.measurement_number);
    if (last_hc_sr04_measurement_number < algo_hc_sr04_data.measurement_number)
    {
        calculate_most_dangerous_sector();
        last_hc_sr04_measurement_number = algo_hc_sr04_data.measurement_number;
    }

    calculate_obstacle_avoidance_angle();

    /**
     * @brief Flag to show that we switched to follow wall in current iteration.
     * Makes so that we do not calculate follow wall angles two times in this case.
     */
    bool switched_to_follow_wall = false;

    switch (algo_obstacle_avoidance_state)
    {
    case OA_BEHAVIOUR_NONE:
        /**
         * @brief Switch to follow the wall behaviour if we found an obstacle.
         */
        if (should_enter_follow_wall_behaviour())
        {
            obstacle_avoidance_state_type new_state = calculate_follow_wall_behaviour();
            algo_obstacle_avoidance_state = new_state;
            switched_to_follow_wall = true;
            follow_wall_ticks = 0;
        }
        break;
    case OA_BEHAVIOUR_FOLLOW_WALL_CLOCKWISE:
    case OA_BEHAVIOUR_FOLLOW_WALL_COUNTERCLOCKWISE:
        if (should_enter_avoid_obstacle_behaviour())
        {
            algo_obstacle_avoidance_state = OA_BEHAVIOUR_AVOID_OBSTACLE;
        }
        if (should_exit_follow_wall_behaviour())
        {
            algo_obstacle_avoidance_state = OA_BEHAVIOUR_NONE;
            algo_follow_wall_angle = INFINITY;
        }
        break;
    case OA_BEHAVIOUR_AVOID_OBSTACLE:
        if (should_exit_avoid_obstacle_behaviour())
        {
            obstacle_avoidance_state_type new_state = calculate_follow_wall_behaviour();
            algo_obstacle_avoidance_state = new_state;
            switched_to_follow_wall = true;
            follow_wall_ticks = 0;
            algo_avoid_obstacle_angle = INFINITY;
        }
    }

    switch (algo_obstacle_avoidance_state)
    {
    case OA_BEHAVIOUR_NONE:
        ESP_LOGI(TAG, "No obstacle found, skipping obstacle avoidance");
        break;
    case OA_BEHAVIOUR_FOLLOW_WALL_CLOCKWISE:
        /**
         * @brief Angle was not calculated, calculate it.
         */
        if (!switched_to_follow_wall)
        {
            calculate_follow_wall_cc_angle();
        }
        algo_follow_wall_angle = follow_wall_cc_angle;
        distance_to_goal = goal_heading_distance_to_goal();
        follow_wall_ticks++;
        ESP_LOGI(TAG, "Avoiding obstacle at angle %.2f clockwise, heading to %.2f", angle_to_obstacle * RAD_TO_DEG, algo_follow_wall_angle * RAD_TO_DEG);
        break;
    case OA_BEHAVIOUR_FOLLOW_WALL_COUNTERCLOCKWISE:
        /**
         * @brief Angle was not calculated, calculate it.
         */
        if (!switched_to_follow_wall)
        {
            calculate_follow_wall_ccw_angle();
        }
        algo_follow_wall_angle = follow_wall_ccw_angle;
        distance_to_goal = goal_heading_distance_to_goal();
        follow_wall_ticks++;
        ESP_LOGI(TAG, "Avoiding obstacle at angle %.2f counterclockwise, heading to %.2f", angle_to_obstacle * RAD_TO_DEG, algo_follow_wall_angle * RAD_TO_DEG);
        break;
    case OA_BEHAVIOUR_AVOID_OBSTACLE:
    {
        int angle_sum_number = 0;
        float angle_sum = 0.0;

        for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
        {
            if (algo_hc_sr04_data.distance[i] < DISTANCE_AVOID_THRESHOLD)
            {
                angle_sum += get_angle_to_obstacle_from_sector(i);
                angle_sum_number++;
            }
        }
        angle_sum /= angle_sum_number;
        algo_avoid_obstacle_angle = get_avoidance_angle_from_angle(angle_sum_number);
    }
    }
}

obstacle_avoidance_state_type calculate_follow_wall_behaviour()
{
    calculate_follow_wall_cc_angle();
    calculate_follow_wall_ccw_angle();

    if (most_dangerous_sector < 2)
    {
        return OA_BEHAVIOUR_FOLLOW_WALL_COUNTERCLOCKWISE;
    }

    if (most_dangerous_sector > 5)
    {
        return OA_BEHAVIOUR_FOLLOW_WALL_CLOCKWISE;
    }

    /**
     * @brief If clockwise direction is closer to the desired heading, choose it.
     * Otherwise, choose the counter-clockwise direction.
     */
    if (fabsf((algo_goal_heading - follow_wall_cc_angle)) < fabsf((algo_goal_heading - follow_wall_ccw_angle)))
    {
        return OA_BEHAVIOUR_FOLLOW_WALL_CLOCKWISE;
    }
    else
    {
        return OA_BEHAVIOUR_FOLLOW_WALL_COUNTERCLOCKWISE;
    }
}

void obstacle_avoidance_force_cw()
{
    algo_obstacle_avoidance_state = OA_BEHAVIOUR_FOLLOW_WALL_CLOCKWISE;
}

void obstacle_avoidance_force_ccw()
{
    algo_obstacle_avoidance_state = OA_BEHAVIOUR_FOLLOW_WALL_COUNTERCLOCKWISE;
}

bool should_enter_follow_wall_behaviour()
{
    return (most_dangerous_sector != -1);
}

bool should_exit_follow_wall_behaviour()
{
    float new_distance_to_goal = goal_heading_distance_to_goal();
    /**
     * @brief Exit follow the wall behaviour when:
     * - area around is safe
     * OR
     * - we made progress to goal (distance is now lower)
     * - obstacle is opposite of goal heading from our perspective
     */
    if ((most_dangerous_sector == -1 ||
         ((new_distance_to_goal < distance_to_goal) &&
          (fabsf(algo_goal_heading - obstacle_avoidance_angle) < (M_PI / 2)))) &&
        follow_wall_ticks > 3)
    {
        return true;
    }
    return false;
}

bool should_enter_avoid_obstacle_behaviour()
{
    return algo_hc_sr04_data.distance[most_dangerous_sector] < DISTANCE_AVOID_THRESHOLD;
}

bool should_exit_avoid_obstacle_behaviour()
{
    return algo_hc_sr04_data.distance[most_dangerous_sector] > DISTANCE_AVOID_THRESHOLD;
}

void calculate_obstacle_avoidance_angle()
{
    if (most_dangerous_sector == -1)
    {
        angle_to_obstacle = get_angle_to_obstacle_from_sector(last_most_dangerous_sector);
    }
    else
    {
        angle_to_obstacle = get_angle_to_obstacle_from_sector(most_dangerous_sector);
    }

    /**
     * @todo Not sure if we need this angle safeguard below.
     * Is there some scenario that this is needed/not needed?
     */
    obstacle_avoidance_angle = get_avoidance_angle_from_angle(angle_to_obstacle);
}

float get_angle_to_obstacle_from_sector(int sector)
{
    return algo_current_heading + hc_sr04_sensor_regions[sector] + SECTOR_ANGLE_HALF;
}

float get_avoidance_angle_from_angle(float angle_to_obstacle)
{
    return ANGLE_SAFEGUARD(angle_to_obstacle + M_PI);
}

void calculate_follow_wall_cc_angle()
{
    follow_wall_cc_angle = angle_to_obstacle + (M_PI / 2.0);
}

void calculate_follow_wall_ccw_angle()
{
    follow_wall_ccw_angle = angle_to_obstacle - (M_PI / 2.0);
}

void calculate_most_dangerous_sector()
{
    if (most_dangerous_sector != -1)
    {
        /**
         * @brief Save last most dangerous sector.
         */
        last_most_dangerous_sector = most_dangerous_sector;
    }
    last_most_dangerous_sector_measurement = most_dangerous_sector_measurement;
    most_dangerous_sector = -1;
    most_dangerous_sector_measurement = -1;
    uint32_t lowest_distance = UINT32_MAX;
    for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
    {
        if ((algo_hc_sr04_data.distance[i] < DISTANCE_SAFE_THRESHOLD) && (algo_hc_sr04_data.distance[i] < lowest_distance))
        {
            lowest_distance = algo_hc_sr04_data.distance[i];
            most_dangerous_sector_measurement = i;
        }
    }

    if ((most_dangerous_sector_measurement != -1) && (last_most_dangerous_sector_measurement == -1))
    {
        /**
         * @brief This is the first iteration of detecting this sector as most dangerous.
         * Skip this, consider sector not dangerous.
         * If it still will be dangerous in the next iteration, then we consider it dangerous.
         * This will help one-off measurement glitches from the sensors.
         */
        ESP_LOGI(TAG, "Sector %d dangerous candidate", most_dangerous_sector_measurement);
        most_dangerous_sector = -1;
    }

    else if ((most_dangerous_sector_measurement == -1) && (last_most_dangerous_sector_measurement != -1))
    {
        /**
         * @brief We found this sector dangerous last time, now area is safe.
         * Keep previous value for this iteration only.
         * If area will still be safe in the next iteration, then we consider it safe.
         * This will help one-off measurement glitches from the sensors.
         */
        ESP_LOGI(TAG, "Sector safe request");
        most_dangerous_sector = last_most_dangerous_sector_measurement;
    }
    else if ((abs(most_dangerous_sector_measurement - last_most_dangerous_sector_measurement) < 3))
    {
        ESP_LOGI(TAG, "Keeping same follow wall direction at sector %d", most_dangerous_sector_measurement);
        most_dangerous_sector = most_dangerous_sector_measurement;
    }
    else
    {
        /**
         * @todo Consider switching directions of the follow angle behaviour.
         */
        ESP_LOGI(TAG, "Requesting recalculation of follow wall direction");
        most_dangerous_sector = most_dangerous_sector_measurement;
        algo_obstacle_avoidance_state = OA_BEHAVIOUR_NONE;
    }
}

// void check_left_sector(int sector_to_check, int *safe_sector)
// {
//     int left_sector = sector_to_check;
//     if (is_sector_safe(left_sector))
//     {
//         *safe_sector = left_sector;
//         return;
//     }
// }

// void check_right_sector(int sector_to_check, int *safe_sector)
// {
//     int right_sector = sector_to_check;
//     if (is_sector_safe(right_sector))
//     {
//         *safe_sector = right_sector;
//         return;
//     }
// }

// int calculate_sector_from_heading(float heading)
// {
//     if (heading >= hc_sr04_sensor_regions[1])
//     {
//         return 0;
//     }
//     if (heading <= hc_sr04_sensor_regions[NUMBER_OF_HC_SR04_SENSORS - 1])
//     {
//         return NUMBER_OF_HC_SR04_SENSORS - 1;
//     }

//     for (int i = 1; i < NUMBER_OF_HC_SR04_SENSORS - 1; ++i)
//     {
//         if (heading <= hc_sr04_sensor_regions[i] && heading >= hc_sr04_sensor_regions[i + 1])
//         {
//             return i;
//         }
//     }
//     ESP_LOGE(TAG, "calculate_sector_from_heading error");
//     return -1;
// }

void obstacle_avoidance_reset()
{
    algo_obstacle_avoidance_state = OA_BEHAVIOUR_NONE;
    algo_follow_wall_angle = INFINITY;
    algo_avoid_obstacle_angle = INFINITY;
    last_hc_sr04_measurement_number = 0;
}

// void calculate_danger_intensities()
// {
//     for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
//     {
//         uint32_t distance = algo_hc_sr04_data.distance[i];
//         float near = fuzzy_input_near(distance);
//         float middle = fuzzy_input_middle(distance);
//         float far = fuzzy_input_far(distance);

//         float nominator = near * FUZZY_OUTPUT_NEAR +
//                           middle * FUZZY_OUTPUT_MIDDLE +
//                           far * FUZZY_OUTPUT_FAR;

//         float denominator = near + middle + far;

//         danger_intensities[i] = nominator / denominator;
//     }
// }

// void calculate_danger_levels(uint32_t distance)
// {
//     most_dangerous_sector = -1;
//     biggest_danger_level = -1;
//     for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
//     {
//         float new_danger_level;
//         uint32_t distance = algo_hc_sr04_data.distance[i];

//         // measurement error, put maximum danger level
//         if (distance == 0)
//         {
//             new_danger_level = DANGER_LEVEL_MAX;
//         }

//         // very far away, minimum danger level
//         else if (distance > DANGER_LEVEL_DISTANCE_THRESHOLD)
//         {
//             new_danger_level = DANGER_LEVEL_MIN;
//         }

//         else
//         {
//             new_danger_level = min(DANGER_LEVEL_MAX, (float)DANGER_LEVEL_DISTANCE_THRESHOLD / distance);
//         }

//         /**
//          * @brief If new danger level is higher than before, set it.
//          *
//          * If danger level difference is smaller than DANGER_LEVEL_DECREMENT_PER_TICK,
//          * also set new danger level.
//          *
//          * If danger level difference is larger than DANGER_LEVEL_DECREMENT_PER_TICK,
//          * decrement danger level by DANGER_LEVEL_DECREMENT_PER_TICK.
//          */
//         if (new_danger_level > danger_levels[i] - DANGER_LEVEL_DECREMENT_PER_TICK)
//         {
//             danger_levels[i] = new_danger_level;
//         }
//         {
//             danger_levels[i] -= DANGER_LEVEL_DECREMENT_PER_TICK;
//         }

//         /**
//          * @brief Check if this is new most dangerous sector.
//          */
//         if (new_danger_level > biggest_danger_level)
//         {
//             biggest_danger_level = new_danger_level;
//             most_dangerous_sector = i;
//         }
//     }
// }

// /**
//  * @brief Fuzzifier transforming distance in micrometers to
//  * three fuzzy states - near, middle, far.
//  * @param distance in micrometers
//  * @return float
//  */

// float fuzzy_input_near(uint32_t distance)
// {
//     if (distance < FUZZY_INPUT_BOUNDARY_NEAR_LOW)
//     {
//         return 1;
//     }

//     if (distance >= FUZZY_INPUT_BOUNDARY_NEAR_LOW &&
//         distance < FUZZY_INPUT_BOUNDARY_NEAR_HIGH)
//     {
//         return (float)(FUZZY_INPUT_BOUNDARY_NEAR_HIGH - distance) /
//                (FUZZY_INPUT_BOUNDARY_NEAR_HIGH - FUZZY_INPUT_BOUNDARY_NEAR_LOW);
//     }

//     return 0;
// }

// float fuzzy_input_middle(uint32_t distance)
// {
//     if (distance < FUZZY_INPUT_BOUNDARY_NEAR_LOW)
//     {
//         return 0;
//     }

//     if (distance >= FUZZY_INPUT_BOUNDARY_NEAR_LOW &&
//         distance < FUZZY_INPUT_BOUNDARY_NEAR_HIGH)
//     {
//         return (float)(distance - FUZZY_INPUT_BOUNDARY_NEAR_LOW) /
//                (FUZZY_INPUT_BOUNDARY_NEAR_HIGH - FUZZY_INPUT_BOUNDARY_NEAR_LOW);
//     }

//     if (distance >= FUZZY_INPUT_BOUNDARY_NEAR_HIGH &&
//         distance < FUZZY_INPUT_BOUNDARY_FAR_LOW)
//     {
//         return 1;
//     }

//     if (distance >= FUZZY_INPUT_BOUNDARY_FAR_LOW &&
//         distance < FUZZY_INPUT_BOUNDARY_FAR_HIGH)
//     {
//         return (float)(FUZZY_INPUT_BOUNDARY_FAR_HIGH - distance) /
//                (FUZZY_INPUT_BOUNDARY_FAR_HIGH - FUZZY_INPUT_BOUNDARY_FAR_LOW);
//     }

//     return 0;
// }

// float fuzzy_input_far(uint32_t distance)
// {
//     if (distance < FUZZY_INPUT_BOUNDARY_FAR_LOW)
//     {
//         return 0;
//     }

//     if (distance >= FUZZY_INPUT_BOUNDARY_FAR_LOW &&
//         distance < FUZZY_INPUT_BOUNDARY_FAR_HIGH)
//     {
//         return (float)(distance - FUZZY_INPUT_BOUNDARY_FAR_LOW) /
//                (FUZZY_INPUT_BOUNDARY_FAR_HIGH - FUZZY_INPUT_BOUNDARY_FAR_LOW);
//     }

//     return 1;
// }