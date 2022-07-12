#include "obstacle_avoidance.h"

#include "data_receive.h"

#include "algo.h"
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

#define DISTANCE_AVOID_THRESHOLD (120000)

#define DISTANCE_FOLLOW_WALL_ENTER_THRESHOLD (550000)
// #define DISTANCE_FOLLOW_WALL_ENTER_THRESHOLD (520000)

/**
 * @todo This distance probably should be larger.
 */
#define DISTANCE_FOLLOW_WALL_LEAVE_THRESHOLD (580000)
// #define DISTANCE_FOLLOW_WALL_LEAVE_THRESHOLD d(550000)

#define FOLLOW_WALL_SWITCH_DIRECTION_DIFFERENCE_THRESHOLD (M_PI / 6.0)
// #define DISTANCE_SAFE_THRESHOLD (560000)

// #define SECTOR_NOT_FOUND (UINT8_MAX)

// fuzzy input boundaries
// #define FUZZY_INPUT_BOUNDARY_NEAR_LOW 10000x
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

/**
 * @brief Distances used for weighing obstacle avoidance angle.
 * "High" is the value used for the highest weight (1.0),
 * "Low" is the value used for the lowest weight (0.0).
 * Linear interpolation is used for values inbetween.
 */
#define DISTANCE_OBSTACLE_AVOIDANCE_WEIGHT_HIGH (100000)
#define DISTANCE_OBSTACLE_AVOIDANCE_WEIGHT_LOW (DISTANCE_FOLLOW_WALL_LEAVE_THRESHOLD)

#define A_WEIGHT ((1.0 / (DISTANCE_OBSTACLE_AVOIDANCE_WEIGHT_HIGH - DISTANCE_OBSTACLE_AVOIDANCE_WEIGHT_LOW)))
#define B_WEIGHT (-A_WEIGHT * DISTANCE_OBSTACLE_AVOIDANCE_WEIGHT_LOW)

/**
 * @brief Distances used for scaling follow wall angle.
 * When farther away from the obstacle, do not turn as much.
 * When closer, turn more.
 */
#define DISTANCE_FOLLOW_WALL_SCALE_HIGH (350000)
#define DISTANCE_FOLLOW_WALL_SCALE_LOW (DISTANCE_FOLLOW_WALL_LEAVE_THRESHOLD)

/**
 * (HIGH, 90)
 * (LOW, 0) -> 0 = a * LOW + b -> b = -a*LOW
 */
#define A_FOLLOW_WALL_SCALE ((M_PI / 2.0) / (DISTANCE_FOLLOW_WALL_SCALE_HIGH - DISTANCE_FOLLOW_WALL_SCALE_LOW))
#define B_FOLLOW_WALL_SCALE (-A_FOLLOW_WALL_SCALE * DISTANCE_FOLLOW_WALL_SCALE_LOW)

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
// static int most_dangerous_sector = -1;
// static int most_dangerous_sector_measurement = -1;
// static int last_most_dangerous_sector = -1;
// static int last_most_dangerous_sector_measurement = -1;
// static int obstacle_disappeared_ticks = 0;

static uint32_t hc_sr04_data_filtered[NUMBER_OF_HC_SR04_SENSORS];
static int hc_sr04_ticks_since_dangerous[NUMBER_OF_HC_SR04_SENSORS];
static int hc_sr04_ticks_since_safe[NUMBER_OF_HC_SR04_SENSORS];

/**
 * @todo Fix type
 */
static uint32_t last_hc_sr04_measurement_number = 0;
static uint32_t follow_wall_ticks = 0;
static uint32_t most_dangerous_sector;
static uint32_t lowest_distance;
static int found_dangerous_ticks;
static float distance_to_goal;
static float angle_to_obstacle;
static float follow_wall_cc_angle;
static float follow_wall_ccw_angle;
static float last_obstacle_avoidance_angle;
static float obstacle_avoidance_angle;
static bool follow_wall_change_direction_request;
static bool area_dangerous;
static bool switched_to_follow_wall;

// local function declarations
static obstacle_avoidance_state_type calculate_follow_wall_behaviour();
static void calculate_distance_data();

static void calculate_obstacle_avoidance_angle();
static void calculate_most_dangerous_sector();
static void calculate_follow_wall_cc_angle();
static void calculate_follow_wall_ccw_angle();
static void switch_to_follow_wall();

static bool should_exit_follow_wall_behaviour();
static bool should_enter_follow_wall_behaviour();
static bool should_enter_avoid_obstacle_behaviour();
static bool should_exit_avoid_obstacle_behaviour();
static bool are_sectors_near(int sector1, int sector2);

static float get_angle_to_obstacle_from_sector(int sector);
static float get_avoidance_angle_from_angle(float angle_to_obstacle);
static float get_weight_from_distance(uint32_t distance);
static float get_scaled_rebound_angle(uint32_t distance);

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
    ESP_LOGI(TAG, "high: %.2f", get_weight_from_distance(DISTANCE_OBSTACLE_AVOIDANCE_WEIGHT_HIGH));
    ESP_LOGI(TAG, "200000: %.2f", get_weight_from_distance(200000));
    ESP_LOGI(TAG, "400000: %.2f", get_weight_from_distance(400000));
    ESP_LOGI(TAG, "low: %.2f", get_weight_from_distance(DISTANCE_OBSTACLE_AVOIDANCE_WEIGHT_LOW));
}

void obstacle_avoidance_calculate()
{
    ESP_LOGI(TAG, "last_measurement: %d, now: %d",
             last_hc_sr04_measurement_number,
             algo_hc_sr04_data.measurement_number);
    if (last_hc_sr04_measurement_number < algo_hc_sr04_data.measurement_number)
    {
        calculate_distance_data();
        last_hc_sr04_measurement_number = algo_hc_sr04_data.measurement_number;
    }
    calculate_obstacle_avoidance_angle();

    /**
     * @brief Flag to show that we switched to follow wall in current iteration.
     * Makes so that we do not calculate follow wall angles two times in this case.
     */
    switched_to_follow_wall = false;

    switch (algo_obstacle_avoidance_state)
    {
    case OA_BEHAVIOUR_NONE:
        /**
         * @brief Switch to follow the wall behaviour if we found an obstacle.
         */
        if (should_enter_follow_wall_behaviour())
        {
            ESP_LOGW(TAG, "Entering follow wall behaviour");
            switch_to_follow_wall();
        }
        break;
    case OA_BEHAVIOUR_FOLLOW_WALL_CLOCKWISE:
    case OA_BEHAVIOUR_FOLLOW_WALL_COUNTERCLOCKWISE:
        // if (should_enter_avoid_obstacle_behaviour())
        // {
        //     algo_obstacle_avoidance_state = OA_BEHAVIOUR_AVOID_OBSTACLE;
        //     algo_follow_wall_angle = INFINITY;
        // }
        // else if (should_exit_follow_wall_behaviour())
        if (should_exit_follow_wall_behaviour())
        {
            ESP_LOGW(TAG, "Leaving follow wall behaviour");
            algo_obstacle_avoidance_state = OA_BEHAVIOUR_NONE;
            algo_follow_wall_angle = INFINITY;
        }
        break;
    case OA_BEHAVIOUR_AVOID_OBSTACLE:
        if (should_exit_avoid_obstacle_behaviour())
        {
            if (area_dangerous)
            {
                obstacle_avoidance_state_type new_state = calculate_follow_wall_behaviour();
                algo_obstacle_avoidance_state = new_state;
                switched_to_follow_wall = true;
            }
            else
            {
                algo_obstacle_avoidance_state = OA_BEHAVIOUR_NONE;
            }
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
        follow_wall_ticks++;
        ESP_LOGI(TAG, "Following wall at angle %.2f clockwise, heading to %.2f", angle_to_obstacle * RAD_TO_DEG, algo_follow_wall_angle * RAD_TO_DEG);
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
        follow_wall_ticks++;
        ESP_LOGI(TAG, "Avoiding obstacle at angle %.2f counterclockwise, heading to %.2f", angle_to_obstacle * RAD_TO_DEG, algo_follow_wall_angle * RAD_TO_DEG);
        break;
    case OA_BEHAVIOUR_AVOID_OBSTACLE:
    {
        // int angle_sum_number = 0;
        // float angle_sum = 0.0;

        // for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
        // {
        //     if (algo_hc_sr04_data.distance[i] < DISTANCE_AVOID_THRESHOLD)
        //     {
        //         angle_sum += get_angle_to_obstacle_from_sector(i);
        //         angle_sum_number++;
        //     }
        // }
        // angle_sum /= angle_sum_number;
        // algo_avoid_obstacle_angle = get_avoidance_angle_from_angle(angle_sum);
        algo_avoid_obstacle_angle = obstacle_avoidance_angle;
        ESP_LOGW(TAG, "Avoiding obstacle at weighted angle %.2f", obstacle_avoidance_angle * RAD_TO_DEG);
    }
    }
}

void calculate_distance_data()
{
    lowest_distance = UINT32_MAX;
    most_dangerous_sector = -1;
    for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
    {
        /**
         * @brief Skip glitch readings.
         */
        if (algo_hc_sr04_data.distance[i] < 2000)
        {
            hc_sr04_data_filtered[i] = 2000000;
            continue;
        }

        if (algo_hc_sr04_data.distance[i] < DISTANCE_FOLLOW_WALL_ENTER_THRESHOLD)
        {
            if (hc_sr04_ticks_since_dangerous[i] >= 1)
            {
                /**
                 * @brief We have confirmed that this sector is dangerous,
                 * use the measured distance.
                 */
                // ESP_LOGI(TAG, "Sector %d dangerous", i);
                hc_sr04_data_filtered[i] = algo_hc_sr04_data.distance[i];
            }
            else
            {
                /**
                 * @brief We have not confirmed that this sector is dangerous.
                 * Keep the old distance for now.
                 */
            }
            hc_sr04_ticks_since_dangerous[i]++;
        }
        else
        {
            hc_sr04_ticks_since_dangerous[i] = 0;
        }

        if (algo_hc_sr04_data.distance[i] >= DISTANCE_FOLLOW_WALL_LEAVE_THRESHOLD)
        {
            if (hc_sr04_ticks_since_safe[i] >= 2)
            {
                /**
                 * @brief We have confirmed that this sector is safe,
                 * use the measured distance.
                 */
                // ESP_LOGI(TAG, "Sector %d safe", i);
                hc_sr04_data_filtered[i] = algo_hc_sr04_data.distance[i];
            }
            else
            {
                /**
                 * @brief We have not confirmed that this sector is safe.
                 * Keep the old distance for now.
                 */
            }
            hc_sr04_ticks_since_safe[i]++;
        }
        else
        {
            hc_sr04_ticks_since_safe[i] = 0;
        }

        if (hc_sr04_data_filtered[i] < lowest_distance)
        {
            lowest_distance = hc_sr04_data_filtered[i];
            most_dangerous_sector = i;
        }
    }
}

obstacle_avoidance_state_type calculate_follow_wall_behaviour()
{
    calculate_follow_wall_cc_angle();
    calculate_follow_wall_ccw_angle();

    // if (most_dangerous_sector < 2)
    // {
    //     return OA_BEHAVIOUR_FOLLOW_WALL_COUNTERCLOCKWISE;
    // }

    // if (most_dangerous_sector > 5)
    // {
    //     return OA_BEHAVIOUR_FOLLOW_WALL_CLOCKWISE;
    // }

    /**
     * @brief If clockwise direction is closer to the desired heading, choose it.
     * Otherwise, choose the counter-clockwise direction.
     */

    obstacle_avoidance_state_type new_state;
    /**
     * @brief If one of the follow wall angles are very close to our current heading (30 deg),
     * choose it to avoid oscillations on the edges of the obstacle.
     */
    if (fabsf(algo_current_heading - follow_wall_cc_angle) < (M_PI / 6.0))
    {
        new_state = OA_BEHAVIOUR_FOLLOW_WALL_CLOCKWISE;
    }
    else if (fabsf(algo_current_heading - follow_wall_ccw_angle) < (M_PI / 6.0))
    {
        new_state = OA_BEHAVIOUR_FOLLOW_WALL_COUNTERCLOCKWISE;
    }
    else
    {
        if (fabsf((algo_goal_heading - follow_wall_cc_angle)) < fabsf((algo_goal_heading - follow_wall_ccw_angle)))
        {
            new_state = OA_BEHAVIOUR_FOLLOW_WALL_CLOCKWISE;
        }
        else
        {
            new_state = OA_BEHAVIOUR_FOLLOW_WALL_COUNTERCLOCKWISE;
        }
    }
    return new_state;
}

void obstacle_avoidance_force_cw()
{
    ESP_LOGW(TAG, "Forcing clockwise follow the wall");
    algo_obstacle_avoidance_state = OA_BEHAVIOUR_FOLLOW_WALL_CLOCKWISE;
}

void obstacle_avoidance_force_ccw()
{
    ESP_LOGW(TAG, "Forcing counter-clockwise follow the wall");
    algo_obstacle_avoidance_state = OA_BEHAVIOUR_FOLLOW_WALL_COUNTERCLOCKWISE;
}

bool should_enter_follow_wall_behaviour()
{
    if (!area_dangerous)
    {
        return false;
    }

    bool found_dangerous = false;
    for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
    {
        if (hc_sr04_data_filtered[i] < DISTANCE_FOLLOW_WALL_ENTER_THRESHOLD)
        {
            found_dangerous = true;
            break;
        }
    }

    if (found_dangerous)
    {
        if (found_dangerous_ticks >= 2)
        {
            /**
             * @brief It has been at least 2 ticks of considering some sector
             * dangerous. Switch to follow wall behaviour.
             */
            found_dangerous_ticks = 0;
            return true;
        }
        else
        {
            /**
             * @brief It has been less than 2 ticks of considering some sector
             * dangerous. Increment tick count and keep waiting.
             */
            found_dangerous_ticks++;
            return false;
        }
    }
    else
    {
        found_dangerous_ticks = 0;
        return false;
    }
}

void switch_to_follow_wall()
{
    obstacle_avoidance_state_type new_state = calculate_follow_wall_behaviour();
    algo_obstacle_avoidance_state = new_state;
    switched_to_follow_wall = true;
    distance_to_goal = goal_heading_distance_to_goal();
    follow_wall_ticks = 0;
}

bool should_exit_follow_wall_behaviour()
{
    /**
     * @brief Exit follow the wall behaviour when:
     * - area around is safe
     * OR
     * - we made progress to goal (distance is now lower)
     * - obstacle is opposite of goal heading from our perspective
     * - we have been in follow wall behaviour for at least 3 ticks
     */

    bool area_safe = true;
    for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
    {
        if (hc_sr04_data_filtered[i] < DISTANCE_FOLLOW_WALL_LEAVE_THRESHOLD)
        {
            area_safe = false;
        }
    }
    if (area_safe)
    {
        return true;
    }

    // float new_distance_to_goal = goal_heading_distance_to_goal();
    // if ((new_distance_to_goal < distance_to_goal) &&
    //     fabsf(algo_goal_heading - obstacle_avoidance_angle) < (M_PI / 2) &&
    //     follow_wall_ticks > 3)
    // {
    //     return true;
    // }
    // return false;

    float new_distance_to_goal = goal_heading_distance_to_goal();
    if ((distance_to_goal - new_distance_to_goal > 0.03) &&
        fabsf(algo_goal_heading - obstacle_avoidance_angle) < (M_PI / 2.0))
    {
        return true;
    }
    return false;
}

bool should_enter_avoid_obstacle_behaviour()
{
    bool retval = false;
    for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
    {
        if (hc_sr04_data_filtered[i] < DISTANCE_AVOID_THRESHOLD)
        {
            retval = true;
            break;
        }
    }
    return retval;
}

bool should_exit_avoid_obstacle_behaviour()
{
    bool retval = true;
    for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
    {
        if (hc_sr04_data_filtered[i] < DISTANCE_AVOID_THRESHOLD)
        {
            retval = false;
            break;
        }
    }
    return retval;
}

void calculate_obstacle_avoidance_angle()
{
    last_obstacle_avoidance_angle = obstacle_avoidance_angle;
    float total_weight = 0.0;
    float weighted_angle = 0.0;
    for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
    {
        if (hc_sr04_data_filtered[i] >= DISTANCE_FOLLOW_WALL_LEAVE_THRESHOLD)
        {
            continue;
        }

        float weight = get_weight_from_distance(hc_sr04_data_filtered[i]);
        float angle_to_obstacle = get_angle_to_obstacle_from_sector(i);
        float angle_obstacle_avoidance = get_avoidance_angle_from_angle(angle_to_obstacle);
        // ESP_LOGI(TAG, "Distance: %d, weight: %.2f, angle_to_obstacle: %.2f, angle_obstacle_avoidance: %.2f",
        //  hc_sr04_data_filtered[i], weight, angle_to_obstacle, angle_obstacle_avoidance);

        weighted_angle += angle_obstacle_avoidance * weight;
        total_weight += weight;
    }

    if (total_weight != 0.0)
    {
        weighted_angle /= total_weight;
        obstacle_avoidance_angle = weighted_angle;
        area_dangerous = true;
        ESP_LOGI(TAG, "Obstacle avoidance angle - weighted angle: %.2f, total_weight: %.2f", weighted_angle, total_weight);
    }
    else
    {
        obstacle_avoidance_angle = INFINITY;
        area_dangerous = false;
    }

    /**
     * @brief Check whether we had a big difference in calculated obstacle
     * avoidance angle from last tick.
     * @todo Might lower this threshold if we observe wrong behaviour in high obstacle
     * density environment.
     */
    if ((fabsf(last_obstacle_avoidance_angle - obstacle_avoidance_angle) > FOLLOW_WALL_SWITCH_DIRECTION_DIFFERENCE_THRESHOLD) &&
        (lowest_distance < DISTANCE_FOLLOW_WALL_ENTER_THRESHOLD) && (follow_wall_ticks > 5))
    {
        /**
         * @brief If we are in follow wall behaviour, switch to no behavior,
         * which will trigger the recalculation of follow wall direction.
         */
        if (algo_obstacle_avoidance_state == OA_BEHAVIOUR_FOLLOW_WALL_CLOCKWISE ||
            algo_obstacle_avoidance_state == OA_BEHAVIOUR_FOLLOW_WALL_COUNTERCLOCKWISE)
        {
            switch_to_follow_wall();
        }
    }
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
    float rebound_angle = get_scaled_rebound_angle(lowest_distance);
    float angle_to_obstacle = get_angle_to_obstacle_from_sector(most_dangerous_sector);
    // follow_wall_cc_angle = ANGLE_SAFEGUARD(obstacle_avoidance_angle - M_PI - rebound_angle);
    follow_wall_cc_angle = ANGLE_SAFEGUARD(angle_to_obstacle + rebound_angle);
    // ESP_LOGI(TAG, "Obstacle at angle %.2f, rebound angle %.2f, follow wall CC angle %.2f",
    //  angle_to_obstacle * RAD_TO_DEG, rebound_angle * RAD_TO_DEG, follow_wall_cc_angle * RAD_TO_DEG);
}

void calculate_follow_wall_ccw_angle()
{
    float angle_to_obstacle = get_angle_to_obstacle_from_sector(most_dangerous_sector);
    float rebound_angle = get_scaled_rebound_angle(lowest_distance);
    follow_wall_ccw_angle = ANGLE_SAFEGUARD(angle_to_obstacle - rebound_angle);
    // ESP_LOGI(TAG, "Obstacle at angle %.2f, rebound angle %.2f, follow wall CCW angle %.2f",
    //  angle_to_obstacle * RAD_TO_DEG, rebound_angle * RAD_TO_DEG, follow_wall_ccw_angle * RAD_TO_DEG);
}

float get_scaled_rebound_angle(uint32_t distance)
{
    float angle;
    if (distance <= DISTANCE_FOLLOW_WALL_SCALE_HIGH)
    {
        angle = (M_PI / 2);
    }
    else if ((distance > DISTANCE_FOLLOW_WALL_SCALE_HIGH) &&
             (distance < DISTANCE_FOLLOW_WALL_SCALE_LOW))
    {
        angle = A_FOLLOW_WALL_SCALE * distance + B_FOLLOW_WALL_SCALE;
    }
    else // distance >= DISTANCE_FOLLOW_WALL_SCALE_LOW
    {
        angle = 0.0;
    }
    return angle;
}

bool are_sectors_near(int sector1, int sector2)
{
    return abs(sector1 - sector2) < 4;
}

float get_weight_from_distance(uint32_t distance)
{
    float weight;
    if (distance <= DISTANCE_OBSTACLE_AVOIDANCE_WEIGHT_HIGH)
    {
        weight = 1.0;
    }
    else if ((distance > DISTANCE_OBSTACLE_AVOIDANCE_WEIGHT_HIGH) &&
             (distance < DISTANCE_OBSTACLE_AVOIDANCE_WEIGHT_LOW))
    {
        // weight = A_WEIGHT * distance + B_WEIGHT;
        weight = (1.0 / ((float)(DISTANCE_OBSTACLE_AVOIDANCE_WEIGHT_LOW - DISTANCE_OBSTACLE_AVOIDANCE_WEIGHT_HIGH) *
                         (float)(DISTANCE_OBSTACLE_AVOIDANCE_WEIGHT_LOW - DISTANCE_OBSTACLE_AVOIDANCE_WEIGHT_HIGH))) *
                 ((float)distance - DISTANCE_OBSTACLE_AVOIDANCE_WEIGHT_LOW) * ((float)distance - DISTANCE_OBSTACLE_AVOIDANCE_WEIGHT_LOW);
    }
    else // distance >= DISTANCE_OBSTACLE_AVOIDANCE_WEIGHT_LOW
    {
        weight = 0.0;
    }
    return weight;
}

void obstacle_avoidance_reset()
{
    algo_obstacle_avoidance_state = OA_BEHAVIOUR_NONE;
    algo_follow_wall_angle = INFINITY;
    algo_avoid_obstacle_angle = INFINITY;
    follow_wall_ticks = 0;

    for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
    {
        hc_sr04_data_filtered[i] = 2000000;
        hc_sr04_ticks_since_dangerous[i] = 0;
        hc_sr04_ticks_since_safe[i] = 0;
    }
    last_hc_sr04_measurement_number = 0;
}
