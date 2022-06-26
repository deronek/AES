#include "algo.h"

#include "ble.h"
#include "data_receive.h"
#include "heading_imu.h"
#include "position.h"
#include "goal_heading.h"
#include "final_heading.h"
#include "obstacle_avoidance.h"
#include "photo_encoder.h"
#include "motor.h"
#include "task_utils.h"
#include "border_recoil.h"
#include "hall.h"

#include "reflectance.h"

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
static uint32_t algo_tick_counter = 0;

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
    /**
     * @todo Initialize all algo data structures
     */
    if ((algo_ble_data_queue = xQueueCreate(1, sizeof(algo_ble_data_type))) == NULL)
    {
        abort();
    }

    position_init();
    photo_encoder_init();
    obstacle_avoidance_init();
    heading_imu_init();
    goal_heading_init();
    final_heading_init();
    hall_init();
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
    reflectance_reset();

    algo_tick_counter = 0;
}

TASK algo_main()
{
    algo_prepare();
    algo_create_tasks();

    vTaskDelay(pdMS_TO_TICKS(50));
    algo_running = true;
    app_manager_notify_main(TASK_ID_ALGO, EVENT_STARTED_DRIVING);
    motor_start(goal_heading_angle_to_goal());

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

        if (algo_stop_requested)
        {
            break;
        }

        algo_ble_send();
        algo_tick_counter++;
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
    final_heading_reset();
    obstacle_avoidance_reset();
    hall_reset();
    border_recoil_reset();

    reflectance_reset();

    algo_stop_requested = false;

    ESP_LOGI(TAG, "Algo cleanup complete");
}

void algo_ble_send()
{
    algo_ble_data_type ble_data;

    ble_data.current_heading = algo_current_heading * RAD_TO_DEG;
    ble_data.pos_x = algo_position.x;
    ble_data.pos_y = algo_position.y;
    ble_data.behaviour_state = algo_final_heading_behaviour_state;
    ble_data.goal_heading = algo_goal_heading * RAD_TO_DEG;
    ble_data.follow_wall_heading = algo_follow_wall_angle * RAD_TO_DEG;
    ble_data.avoid_obstacle_heading = algo_avoid_obstacle_angle * RAD_TO_DEG;
    ble_data.final_heading = algo_final_heading * RAD_TO_DEG;

    ble_send_from_task(TASK_ID_ALGO, &ble_data);
}

void algo_run()
{
    /**
     * @brief Enable reflectance measurement only after 50 ticks (~5s).
     */
    if (algo_tick_counter == 50)
    {
        reflectance_measurement_enabled = true;
    }

    hall_measure();
    if (algo_hall_detected)
    {
        ESP_LOGI(TAG, "Hall sensor triggered, stopping");
        algo_stop_requested = true;
        return;
    }

    border_recoil_calculate();

    heading_imu_calculate();
    goal_heading_calculate();
    obstacle_avoidance_calculate();

    final_heading_calculate();

    motor_control_input_data_type motor_control;
    motor_control.current_heading = algo_current_heading;
    motor_control.desired_heading = algo_final_heading;
    // motor_control.desired_heading = algo_goal_heading;

    motor_tick(motor_control);
}

void algo_request_stop()
{
    algo_stop_requested = true;
}