#include "app_manager.h"

// constants
#define TASK_TICK_PERIOD pdMS_TO_TICKS(100)

// global variables
app_manager_state_type app_manager_state;

const uint8_t app_manager_task_data_size[] = {
    sizeof(hc_sr04_data_type),
    sizeof(mpu9255_fifo_data_type)};

// local variables
static const char *TAG = "app_manager";

TaskHandle_t app_manager_algo_task_handle,
    app_manager_mpu9255_task_handle,
    app_manager_main_task_handle,
    app_manager_hc_sr04_task_handle,
    app_manager_ble_task_handle;

// function declarations
static void app_manager_run();
static void app_manager_update_state();
static void app_manager_init_peripherals();
static void app_manager_create_ble_task();
static void app_manager_create_sensor_tasks();
static void app_manager_create_main_task();
static void app_manager_create_algo_task();
inline static void log_abort_wrong_state(const char *state);

// function definitions
inline static void log_abort_wrong_state(const char *state)
{
    ESP_LOGE(TAG, "app_manager_state == %s in main loop", state);
    abort();
}

TASK app_manager_init()
{
    app_manager_state = APP_MANAGER_INIT;

    ESP_LOGI(TAG, "Core ID: %d", xPortGetCoreID());
    app_manager_init_peripherals();
    app_manager_create_ble_task();
    app_manager_create_sensor_tasks();
    app_manager_create_main_task();
    app_manager_create_algo_task();

    // estabilish communication with user here
    app_manager_state = APP_MANAGER_READY;

    // ESP_LOGI(TAG, "App manager initialization successful");
    vTaskDelete(NULL);
    abort();
}

void app_manager_init_peripherals()
{
    // i2c_master_init();
    // mpu9255_init();
    hc_sr04_init();
}

void app_manager_create_ble_task()
{
    task_utils_create_task(
        ble_main,
        "ble_main",
        2048,
        NULL,
        3,
        &app_manager_ble_task_handle,
        0);
}

void app_manager_create_sensor_tasks()
{
    // TODO: implement some scalable interface for creating tasks
    /**
     * @brief MPU9255 sensor task
     */
    /*
    task_utils_create_task(
        mpu9255_task_measure,
        "mpu9255_task_measure",
        2048,
        NULL,
        4,
        &app_manager_mpu9255_task_handle,
        0);
*/
    /**
     * @brief HC-SR04 sensors task
     */
    task_utils_create_task(
        hc_sr04_measure,
        "hc_sr04_measure",
        2048,
        NULL,
        4,
        &app_manager_hc_sr04_task_handle,
        1);
}

void app_manager_create_main_task()
{
    task_utils_create_task(
        app_manager_main,
        "app_manager_main",
        2048,
        NULL,
        2,
        &app_manager_main_task_handle,
        0);
}

void app_manager_create_algo_task()
{
    task_utils_create_task(
        algo_main,
        "algo_main",
        2048,
        NULL,
        3,
        &app_manager_algo_task_handle,
        0);
}

TASK app_manager_main()
{
    char *buffer = malloc(400);
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;)
    {
        app_manager_run();
        app_manager_update_state();
        vTaskGetRunTimeStats(buffer);
        printf("%s", buffer);
        vTaskDelay(pdMS_TO_TICKS(5000));
        // task_utils_sleep_or_warning(&last_wake_time, TASK_TICK_PERIOD, TAG);
    }
}

void app_manager_run()
{
    switch (app_manager_state)
    {
    case APP_MANAGER_INIT:
        log_abort_wrong_state("APP_MANAGER_INIT");
        abort();
        break;
    case APP_MANAGER_READY:
        // probably nothing to do, just waiting for user
        break;
    case APP_MANAGER_CALIBRATING:
        // probably nothing to do, wait for calibration end
        break;
    case APP_MANAGER_DRIVING:
        algo_run();
        ESP_LOGI(TAG, "Yaw: %.1f", algo_euler_angles.yaw * 180.0 / M_PI);
        /*
            - wait for all sensor data:
                - ultrasonic sensor
                - laser sensor
                - ToF sensor
                - IMU
                - etc.
            - perform algo run
            - drive motors accordingly
        */
        break;
    case APP_MANAGER_FINISHED:
        // probably nothing to do
        break;
    case APP_MANAGER_FAIL:
        // stay in this state, or go back to APP_MANAGER_READY if user requests
        break;
    }
}

void app_manager_update_state()
{
    switch (app_manager_state)
    {
    case APP_MANAGER_INIT:
        log_abort_wrong_state("APP_MANAGER_INIT");
        abort();
    case APP_MANAGER_READY:
        // if GO signal, move to APP_MANAGER_CALIBRATING
        break;
    case APP_MANAGER_CALIBRATING:
        // if calibration ended, move to APP_MANAGER_DRIVING
        break;
    case APP_MANAGER_DRIVING:
        /*
            if:
            - end line detected
            - STOP signal from user
            - other end condition
            probably just move to APP_MANAGER_FINISHED
        */
        break;
    case APP_MANAGER_FINISHED:
        // stay in this state, or go back to APP_MANAGER_READY if user requests
        break;
    case APP_MANAGER_FAIL:
        // stay in this state, or go back to APP_MANAGER_READY if user requests
        break;
    }
}