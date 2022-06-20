#include "algo.h"

#include "ble.h"
#include "data_receive.h"
#include "heading_imu.h"
#include "position.h"
#include "desired_heading.h"
#include "photo_encoder.h"
#include "motor.h"
#include "hall.h"

// constants

// #define COMP_FILTER_ALPHA 0.5

// global variables
// algo_quaternion_type algo_quaternion;
// algo_euler_angles_type algo_euler_angles;
// algo_heading_data_type algo_current_heading;
// QueueHandle_t algo_heading_data_queue;
TaskHandle_t algo_position_process_task_handle,
    algo_position_photo_encoder_process_task_handle,
    algo_position_accel_process_task_handle;
QueueHandle_t algo_ble_data_queue;

/**b
 * @todo Refactor this with getter
 */
bool algo_running = false;
bool algo_stop_requested = false;

// local variables
/**
 * @brief Heading of finish line in degress;
 * @todo Make this value dynamic from controller app.
 */
static const char *TAG = "algo";

static float finish_heading = 30.0;

float phiHat_rad = 0.0f;
float thetaHat_rad = 0.0f;

// constants
#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)
#define TASK_TICK_PERIOD TASK_HZ_TO_TICKS(ALGO_FREQUENCY)

// function declarations
static void algo_ble_send();
static void algo_prepare();
static void algo_create_tasks();
// static void algo_update_quaternion();
// static float algo_low_pass_tick(float xn);
// static float algo_high_pass_tick(float xn);
static void algo_cleanup();

// function definitions
void algo_init()
{
    // hall_init();
    /**
     * @todo Initialize all algo data structures
     */
    if ((algo_ble_data_queue = xQueueCreate(1, sizeof(algo_ble_data_type))) == NULL)
    {
        abort();
    }

    position_init();
    photo_encoder_init();
    heading_imu_init();
    desired_heading_init();
    motor_init();
}

void algo_create_tasks()
{
    task_utils_create_task(
        position_accel_process,
        "position_accel_process",
        4096,
        NULL,
        6,
        &algo_position_accel_process_task_handle,
        1);

    task_utils_create_task(
        position_photo_encoder_process,
        "position_photo_encoder_process",
        4096,
        NULL,
        5,
        &algo_position_photo_encoder_process_task_handle,
        0);

    task_utils_create_task(
        position_process,
        "position_process",
        4096,
        NULL,
        5,
        &algo_position_process_task_handle,
        1);
}

static void algo_prepare()
{
    /**
     * @brief MPU9255
     */
    mpu_reset_fifo();
    ESP_ERROR_CHECK(mpu9255_set_isr(true));
    mpu9255_get_calibration();

    ESP_LOGI(TAG, "Resetting DMP");
    ESP_ERROR_CHECK(mpu9255_set_isr(false));
    mpu9255_reset();
    mpu9255_apply_calibration();
    ESP_ERROR_CHECK(mpu9255_set_isr(true));
    ESP_LOGI(TAG, "DMP reset");

    /**
     * @brief Photo encoder
     */
    photo_encoder_enable_isr();
}

TASK algo_main()
{
    algo_prepare();
    algo_create_tasks();

    vTaskDelay(pdMS_TO_TICKS(50));
    algo_running = true;
    app_manager_notify_main(TASK_ID_ALGO, EVENT_STARTED_DRIVING);

    // get start time of this iteration
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;)
    {
        if (algo_stop_requested)
        {
            break;
        }
        // wait for notification, which will be sent by any of the sensor tasks
        // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /**
         * @todo Implement to only get data which actually updated,
         * so we do not have to copy every one of it.
         * Can probably implemented using bit flags in task notification value.
         */
        // update algo data from sensors
        algo_data_receive_get_latest();

        algo_run();
        algo_ble_send();
        /**
         * @brief We want to run algo calculations at least with ALGO_FREQUENCY.
         * If we do not meet that time, signal a warning.
         */
        task_utils_sleep_or_warning(&last_wake_time, TASK_TICK_PERIOD, TAG);
    }

    algo_cleanup();
    algo_running = false;

    /**
     * @brief Notify about driving state.
     */
    app_manager_notify_main(TASK_ID_ALGO, EVENT_FINISHED_DRIVING);

    /**
     * @brief Notify that task is ready to be deleted.
     */
    xTaskNotifyGive(app_manager_main_task_handle);

    vTaskSuspend(NULL);
}

void algo_cleanup()
{
    ESP_LOGI(TAG, "Algo cleanup requested");
    /**
     * @brief Stop the motors first.
     */
    motor_reset();
    /**
     * @brief Request stop and delete all internal algo tasks.
     */
    task_utils_request_delete_task(&algo_position_process_task_handle,
                                   position_process_request_stop);
    task_utils_request_delete_task(&algo_position_accel_process_task_handle,
                                   position_accel_process_request_stop);
    task_utils_request_delete_task(&algo_position_photo_encoder_process_task_handle,
                                   position_photo_encoder_process_request_stop);
    /**
     * @brief Stop receiving data from MPU9255 and reset accel data queue
     * to be empty for next algo start.
     */
    ESP_ERROR_CHECK(mpu9255_set_isr(false));
    vTaskDelay(pdMS_TO_TICKS(10));
    xQueueReset(mpu9255_queue_accel_data);

    /**
     * @brief Stop photo encoder ISR.
     */
    photo_encoder_disable_isr();

    position_reset();
    heading_imu_reset();

    algo_stop_requested = false;

    ESP_LOGI(TAG, "Algo cleanup complete");
}

void algo_ble_send()
{
    algo_ble_data_type ble_data;

    ble_data.heading = algo_current_heading * RAD_TO_DEG;
    ble_data.pos_x = algo_position.x;
    ble_data.pos_y = algo_position.x;

    ble_send_from_task(TASK_ID_ALGO, &ble_data);
}

void algo_run()
{
    heading_imu_calculate();
    desired_heading_calculate();
    // obstacle_avoidance_calculate();

    motor_control_input_data_type motor_control;
    motor_control.current_heading = algo_current_heading;
    motor_control.desired_heading = algo_desired_heading;

    motor_tick(motor_control);

    /**
     * @todo Send big algo packet (with every calculated data)
     * via BLE here.
     */

    // float gyro_sens;
    // mpu_get_gyro_sens(&gyro_sens);

    // unsigned short accel_sens;
    // mpu_get_accel_sens(&accel_sens);

    // float gyro_x = (float)algo_mpu9255_quaternion_data.gyro.x / gyro_sens;
    // float gyro_y = (float)algo_mpu9255_quaternion_data.gyro.y / gyro_sens;
    // float gyro_z = (float)algo_mpu9255_quaternion_data.gyro.z / gyro_sens;

    // float accel_x = (float)algo_mpu9255_quaternion_data.accel.x / accel_sens;
    // float accel_y = (float)algo_mpu9255_quaternion_data.accel.y / accel_sens;
    // float accel_z = (float)algo_mpu9255_quaternion_data.accel.z / accel_sens;

    // CompGyroUpdate(&filter, gyro_x, gyro_y, gyro_z);
    // CompAccelUpdate(&filter, accel_x, accel_y, accel_z);

    // CompUpdate(&filter);

    // float x, y;
    // CompAnglesGet(&filter, &x, &y);

    // ESP_LOGI(TAG, "%.2f %.2f", x * RAD_TO_DEG, y * RAD_TO_DEG);

    // float phiHat_acc_rad = atanf(accel_y / accel_z);
    // float thetaHat_acc_rad = asinf(accel_x / 9.81f);

    // float phiDot_rps = gyro_x + tanf(thetaHat_rad) * (sinf(phiHat_rad) * gyro_y + cosf(phiHat_rad) * gyro_z);
    // float thetaDot_rps = cosf(phiHat_rad) * gyro_y - sinf(phiHat_rad) * gyro_z;

    // // phiHat_rad = phiHat_rad + (0.1f) * phiDot_rps;
    // // thetaHat_rad = thetaHat_rad + (0.1f) * thetaDot_rps;

    // phiHat_rad = COMP_FILTER_ALPHA * phiHat_acc_rad + (1.0f - COMP_FILTER_ALPHA) * (phiHat_rad + (0.1f) * phiDot_rps);

    // thetaHat_rad = COMP_FILTER_ALPHA * thetaHat_acc_rad + (1.0 - COMP_FILTER_ALPHA) * (thetaHat_rad + (0.1f) * thetaDot_rps);

    // ESP_LOGI(TAG, "%.3f %.3f", phiHat_rad * RAD_TO_DEG, thetaHat_rad * RAD_TO_DEG);

    // algo_update_quaternion();
    // yaw

    // // roll
    // float sinr_cosp = 2 * (algo_quaternion.w * algo_quaternion.x + algo_quaternion.y * algo_quaternion.z);
    // float cosr_cosp = 1 - 2 * (algo_quaternion.x * algo_quaternion.x + algo_quaternion.y * algo_quaternion.y);
    // algo_euler_angles.roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    // // pitch
    // float sinp = 2 * (algo_quaternion.w * algo_quaternion.y - algo_quaternion.z * algo_quaternion.x);
    // if (abs(sinp) >= 1)
    // {
    //     ESP_LOGW(TAG, "Pitch out of range");
    //     algo_euler_angles.pitch = copysignf(M_PI / 2, sinp);
    // }
    // else
    // {
    //     algo_euler_angles.pitch = asinf(sinp);
    // }

    // float mag_x = algo_mpu9255_quaternion_data.mag.x;
    // float mag_y = algo_mpu9255_quaternion_data.mag.y;

    // float yaw_mag = atan2f(mag_y, mag_x);
    // float declination = 8.133 * DEG_TO_RAD;
    // yaw_mag += declination;

    // float yaw_complimentary = algo_low_pass_tick(yaw_mag) + algo_high_pass_tick(yaw_quat);
    // yaw_complimentary *= RAD_TO_DEG;

    // ESP_LOGI(TAG, "Yaw: %.2f", yaw_complimentary);

    /*

        algo_heading_data_type algo_current_heading;
        algo_current_heading.heading = (int16_t)(yaw_quat * 10);

        xQueueOverwrite(algo_heading_data_queue, &algo_current_heading);
        app_manager_ble_task_notify(TASK_FLAG_ALGO);
    */

    // float yaw = atanf((-mag_y * cosf(algo_euler_angles.roll) + mag_z * sinf(algo_euler_angles.roll)) / (mag_x * cosf(algo_euler_angles.pitch) + mag_y * sinf(algo_euler_angles.roll) * sinf(algo_euler_angles.pitch) + mag_z * cosf(algo_euler_angles.roll) * sinf(algo_euler_angles.pitch)));

    // ESP_LOGI(TAG, "Yaw: %.2f", algo_euler_angles.yaw);
    // ESP_LOGI(TAG, "Mag deg: %.2f", deg);
    // ESP_LOGI(TAG, "Yaw: %.2f", yaw * RAD_TO_DEG);

    // ESP_LOGI(TAG, "Gyro: %hi %hi %hi", algo_mpu9255_quaternion_data.gyro.x, algo_mpu9255_quaternion_data.gyro.y, algo_mpu9255_quaternion_data.gyro.z);
    // ESP_LOGI(TAG, "Mag: %hi %hi %hi", algo_mpu9255_quaternion_data.mag.x, algo_mpu9255_quaternion_data.mag.y, algo_mpu9255_quaternion_data.mag.z);
}

// float algo_high_pass_tick(float xn)
// {
//     static float yn_1 = 0;
//     static float xn_1 = 0;

//     float yn = COMP_FILTER_ALPHA * yn_1 + COMP_FILTER_ALPHA * (xn_1 - xn);

//     yn_1 = yn;
//     xn_1 = xn;

//     return yn;
// }

// float algo_low_pass_tick(float xn)
// {
//     static float yn_1 = 0;

//     float yn = (1 - COMP_FILTER_ALPHA) * xn + COMP_FILTER_ALPHA * yn_1;

//     yn_1 = yn;

//     return yn;
// }

void algo_request_stop()
{
    algo_stop_requested = true;
}