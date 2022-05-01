#include "algo.h"
#include "data_receive.h"

// global variables
algo_quaternion_type algo_quaternion;
algo_euler_angles_type algo_euler_angles;

// local variables
static const char *TAG = "algo";

// constants
#define ALGO_FREQUENCY 10
#define TASK_TICK_PERIOD TASK_HZ_TO_TICKS(ALGO_FREQUENCY)

#define QUATERNION_SCALE_FACTOR (2 << 29)

// function declarations
void algo_update_quaternion();

// function definitions

void algo_update_quaternion()
{
    algo_quaternion.w = (float)algo_mpu9255_fifo_data.quaternion.w / QUATERNION_SCALE_FACTOR;
    algo_quaternion.x = (float)algo_mpu9255_fifo_data.quaternion.x / QUATERNION_SCALE_FACTOR;
    algo_quaternion.y = (float)algo_mpu9255_fifo_data.quaternion.y / QUATERNION_SCALE_FACTOR;
    algo_quaternion.z = (float)algo_mpu9255_fifo_data.quaternion.z / QUATERNION_SCALE_FACTOR;
}

TASK algo_main()
{
    // get start time of this iteration
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;)
    {
        // wait for notification, which will be sent by any of the sensor tasks
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /**
         * @todo Implement to only get data which actually updated,
         * so we do not have to copy every one of it.
         * Can probably implemented using bit flags in task notification value.
         */
        // update algo data from sensors
        algo_data_receive_get_latest();

        // run algo calculations
        algo_run();

        /**
         * @brief We want to run algo calculations at least with ALGO_FREQUENCY.
         * If we do not meet that time, signal a warning.
         */
        // task_utils_sleep_or_warning(&last_wake_time, TASK_TICK_PERIOD, TAG);
    }
}

void algo_run()
{
    algo_update_quaternion();
    // yaw
    float siny_cosp = 2 * (algo_quaternion.w * algo_quaternion.z +
                           algo_quaternion.x * algo_quaternion.y);
    float cosy_cosp = 1 - 2 * (algo_quaternion.y * algo_quaternion.y +
                               algo_quaternion.z * algo_quaternion.z);
    algo_euler_angles.yaw = atan2(siny_cosp, cosy_cosp) * (180.0 / M_PI);

    ESP_LOGI(TAG, "Yaw: %.2f", algo_euler_angles.yaw);
}