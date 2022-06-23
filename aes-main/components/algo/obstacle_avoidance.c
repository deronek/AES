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
#define DANGER_LEVEL_DISTANCE_THRESHOLD 600000

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

// local variables
static const char *TAG = "algo-obstacle-avoidance";
float danger_intensities[NUMBER_OF_HC_SR04_SENSORS];
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
     * @brief Danger level defines how much corrective action
     * the robot has to take to avoid collision.
     */
    calculate_danger_levels();

    ESP_LOGI(TAG, "Danger levels: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
             danger_levels[0],
             danger_levels[1],
             danger_levels[2],
             danger_levels[3],
             danger_levels[4],
             danger_levels[5],
             danger_levels[6],
             danger_levels[7]);

    int current_heading_sector = calculate_sector_from_heading(algo_current_heading);
    int desired_heading_sector = calculate_sector_from_heading(algo_desired_heading);
    ESP_LOGI(TAG, "Current sector: %d, desired sector: %d", current_heading_sector, desired_heading_sector);

    /**
     * @brief Calculate to which sector should we drive to to avoid collision.
     * Value SECTOR_NOT_FOUND = UINT8_MAX means we have not found that sector (yet).
     */
    int safe_sector = SECTOR_NOT_FOUND;

    /**
     * @brief If desired heading sector is safe, choose it.
     */
    if (is_sector_safe(desired_heading_sector))
    {
        safe_sector = desired_heading_sector;
    }
    /**
     * @brief If desired heading sector is not safe, find another safe sector.
     * First candidates are sector on the both sides of the desired heading sector.
     * We choose one which is closer to current heading.
     */
    else
    {
        bool prefer_left = (algo_desired_heading > algo_current_heading) ? false : true;

        if (prefer_left)
        {
            for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
            {
                // ESP_LOGI(TAG, "Prefer left, checking sector offset %d", i);
                check_left_sector(desired_heading_sector - i, &safe_sector);
                if (safe_sector != SECTOR_NOT_FOUND)
                {
                    break;
                }
                check_right_sector(desired_heading_sector + i, &safe_sector);
                if (safe_sector != SECTOR_NOT_FOUND)
                {
                    break;
                }
            }
        }
        else
        {
            for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
            {
                check_right_sector(desired_heading_sector + i, &safe_sector);
                // ESP_LOGI(TAG, "Prefer right, checking sector offset %d", i);
                if (safe_sector != SECTOR_NOT_FOUND)
                {
                    break;
                }
                check_left_sector(desired_heading_sector - i, &safe_sector);
                if (safe_sector != SECTOR_NOT_FOUND)
                {
                    break;
                }
            }
        }
    }

    /**
     * @brief If we found a safe sector, point to the center of the safe area.
     */
    float safe_region_center_angle;
    if (safe_sector != SECTOR_NOT_FOUND)
    {
        int safe_to_left = 0;
        int safe_to_right = 0;
        bool search_to_left = true;
        bool search_to_right = true;

        /**
         * @brief Check sectors to the left and right of the safe sector.
         */
        for (int offset = 1; offset < NUMBER_OF_HC_SR04_SENSORS && (search_to_left || search_to_right); ++offset)
        {
            // ESP_LOGI(TAG, "Checking for safe region boundaries, offset %d", offset);
            if (search_to_left && is_sector_safe(safe_sector - offset))
            {
                safe_to_left++;
            }
            else
            {
                search_to_left = false;
            }

            if (search_to_right && is_sector_safe(safe_sector + offset))
            {
                safe_to_right++;
            }
            else
            {
                search_to_right = false;
            }
        }
        // ESP_LOGI(TAG, "Safe area: %d to the left, %d to the right from sector %d", safe_to_left, safe_to_right, safe_sector);
        safe_region_center_angle = (hc_sr04_sensor_regions[safe_sector - safe_to_left] + hc_sr04_sensor_regions[safe_sector + safe_to_right]) / 2.0 + SECTOR_ANGLE_HALF;
        // ESP_LOGI(TAG, "Safe region center angle: %.2f", safe_region_center_angle);
    }
    else
    {
        ESP_LOGE(TAG, "No free heading sector");
        algo_obstacle_avoidance_heading_sector = 255;
        return;
    }

    /**
     * @brief Transform heading sector into actual angle.
     * We aim for the centre of the sector.
     */
    algo_obstacle_avoidance_heading_sector = safe_sector;
    algo_obstacle_avoidance_steering_angle = algo_current_heading + safe_region_center_angle;
    algo_obstacle_avoidance_danger_level_in_heading = danger_levels[safe_sector];

    ESP_LOGI(TAG, "Choosing sector %d, angle %.2f, danger level %d",
             algo_obstacle_avoidance_heading_sector,
             algo_obstacle_avoidance_steering_angle * RAD_TO_DEG,
             algo_obstacle_avoidance_danger_level_in_heading);

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