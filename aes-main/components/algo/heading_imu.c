#include "heading_imu.h"

// global variables
float heading_imu_current = 0.0;

void heading_imu_init()
{
}

void heading_imu_calculate()
{
    /**
     * @todo Calculate current heading from MPU9255 gyroscope + magnetometer data
     * and fill heading_current variable.
     */
    heading_imu_current = 20.0;
}