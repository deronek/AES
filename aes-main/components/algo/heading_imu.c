#include "heading_imu.h"

#include "data_receive.h"

#include <math.h>

// constants
#define QUATERNION_SCALE_FACTOR (2 << 29)
#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)

// structs
typedef struct algo_quaternion_type_tag
{
    float w;
    float x;
    float y;
    float z;
} algo_quaternion_type;

// global variables
float algo_current_heading;

// local variables
static const char *TAG = "algo-heading-imu";

algo_quaternion_type algo_quaternion;

void heading_imu_init()
{
}

void heading_imu_update_quaternion()
{
    algo_quaternion.w = (float)algo_mpu9255_quaternion_data.w / QUATERNION_SCALE_FACTOR;
    algo_quaternion.x = (float)algo_mpu9255_quaternion_data.x / QUATERNION_SCALE_FACTOR;
    algo_quaternion.y = (float)algo_mpu9255_quaternion_data.y / QUATERNION_SCALE_FACTOR;
    algo_quaternion.z = (float)algo_mpu9255_quaternion_data.z / QUATERNION_SCALE_FACTOR;
}

void heading_imu_calculate()
{
    heading_imu_update_quaternion();
    /**
     * @todo Calculate current heading from MPU9255 gyroscope + magnetometer data
     * using complementary/Kalman/Extended Kalman filter.
     */
    float siny_cosp = 2 * (algo_quaternion.w * algo_quaternion.z +
                           algo_quaternion.x * algo_quaternion.y);
    float cosy_cosp = 1 - 2 * (algo_quaternion.y * algo_quaternion.y +
                               algo_quaternion.z * algo_quaternion.z);
    float heading = atan2f(siny_cosp, cosy_cosp);
    algo_current_heading = heading;

    // ESP_LOGI(TAG, "Heading: %.2f", heading);
}

void heading_imu_reset()
{
}