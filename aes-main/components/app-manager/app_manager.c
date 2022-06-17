#include "app_manager.h"
#include "position.h"
#include "photo_encoder.h"

#include "driver/gpio.h"
#include "freertos/task.h"

// constants
#define TASK_TICK_PERIOD pdMS_TO_TICKS(100)

// global variables
app_manager_state_type app_manager_state = APP_MANAGER_INIT;
QueueHandle_t app_manager_event_queue;

const uint8_t app_manager_task_data_size[] = {
    sizeof(hc_sr04_data_type),
    sizeof(mpu9255_quaternion_data_type),
    sizeof(algo_ble_data_type)};

// local variables
static const char *TAG = "app_manager";

TaskHandle_t app_manager_algo_task_handle,
    app_manager_mpu9255_task_handle,
    app_manager_main_task_handle,
    app_manager_hc_sr04_task_handle,
    app_manager_ble_main_task_handle,
    app_manager_ble_heartbeat_task_handle,
    app_manager_init_task_handle;

// function declarations
static void app_manager_run();
static void app_manager_update_state();
static void app_manager_init_peripherals();
static void app_manager_i2c_init();
static void app_manager_create_ble_main_task();
static void app_manager_create_ble_heartbeat_task();
static void app_manager_create_sensor_tasks();
static void app_manager_create_main_task();
static void app_manager_create_algo_task();
static void app_manager_task_notify();

// function definitions
inline static void log_abort_wrong_state(const char *state)
{
    ESP_LOGE(TAG, "app_manager_state == %s in main loop", state);
    abort();
}
static void app_manager_task_notify(TaskHandle_t task_handle, app_manager_task_flag_type task_flag)
{
    xTaskNotify(task_handle, task_flag, eSetBits);
}

TASK app_manager_init()
{
    // ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVELMASK));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    ESP_LOGI(TAG, "Core ID: %d", xPortGetCoreID());
    app_manager_init_peripherals();
    app_manager_create_ble_main_task();
    app_manager_create_ble_heartbeat_task();
    app_manager_create_sensor_tasks();
    app_manager_create_main_task();

    /**
     * @brief Create app manager event queue, used for communicating with
     * algo and BLE modules.
     */
    app_manager_event_queue = xQueueCreate(5, sizeof(app_manager_event_type));
    if (app_manager_event_queue == NULL)
    {
        abort();
    }

    // estabilish communication with user here
    app_manager_state = APP_MANAGER_READY;

    // ESP_LOGI(TAG, "App manager initialization successful");
    vTaskDelete(NULL);
    abort();
}

void app_manager_init_peripherals()
{
    algo_init();
    app_manager_i2c_init();
    // i2c_master_init();

    mpu9255_init();
    // hc_sr04_init();
    ble_init();
}

void app_manager_i2c_init()
{
    TaskHandle_t task_handle;

    /**
     * @brief I2C init function must be executed on core which it operates on.
     * If I2C handler is registered on another core, the other core is stuck in spinlock.
     * Create the task and wait for it to notify the app_manager_init task
     * that the initialization is complete, then delete the task.
     */
    task_utils_create_task(
        i2c_master_init,
        "i2c_master_init",
        4096,
        NULL,
        5,
        &task_handle,
        1);
    task_utils_request_delete_task(&task_handle, NULL);
}

void app_manager_create_ble_main_task()
{
    task_utils_create_task(
        ble_main,
        "ble_main",
        4096,
        NULL,
        3,
        &app_manager_ble_main_task_handle,
        0);
}

void app_manager_create_ble_heartbeat_task()
{
    task_utils_create_task(
        ble_heartbeat,
        "ble_heartbeat",
        2048,
        NULL,
        3,
        &app_manager_ble_heartbeat_task_handle,
        0);
}

void app_manager_create_sensor_tasks()
{
    // TODO: implement some scalable interface for creating tasks
    /**
     * @brief MPU9255 sensor task
     */

    task_utils_create_task(
        mpu9255_task_measure,
        "mpu9255_task_measure",
        4096,
        NULL,
        7,
        &app_manager_mpu9255_task_handle,
        1);

    /**
     * @brief HC-SR04 sensors task
     */
    // task_utils_create_task(
    //     hc_sr04_measure,
    //     "hc_sr04_measure",
    //     8192,
    //     NULL,
    //     4,
    //     &app_manager_hc_sr04_task_handle,
    //     1);
}

void app_manager_create_main_task()
{
    task_utils_create_task(
        app_manager_main,
        "app_manager_main",
        4096,
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
        4096,
        NULL,
        4,
        &app_manager_algo_task_handle,
        0);
}

void app_manager_start_driving()
{
    app_manager_create_algo_task();
    app_manager_state = APP_MANAGER_DRIVING;
}

void app_manager_stop_driving()
{
    if (app_manager_algo_task_handle != NULL)
    {
        task_utils_request_delete_task(&app_manager_algo_task_handle, algo_request_stop);
    }
    else
    {
        ESP_LOGE(TAG, "Algo task handle empty");
    }
}

TASK app_manager_main()
{
    char *buffer = malloc(800);
    if (buffer == NULL)
    {
        abort();
    }
    // for (;;)
    // {
    //     app_manager_run();
    //     app_manager_update_state();
    //     task_utils_sleep_or_warning(&last_wake_time, TASK_TICK_PERIOD, TAG);
    // }

    // vTaskDelay(20000);
    // app_manager_start_driving();
    // vTaskDelay(pdMS_TO_TICKS(60000));
    // algo_request_stop();
    // vTaskDelay(pdMS_TO_TICKS(5000));

    // vTaskGetRunTimeStats(buffer);
    // printf("%s", buffer);
    for (;;)
    {
        // vTaskDelay(5000);
        app_manager_event_type event;
        xQueueReceive(app_manager_event_queue, &event, portMAX_DELAY);

        /**
         * @todo Check app manager state here and handle response to controller app.
         */
        switch (event.type)
        {
        case EVENT_REQUEST_START:
            if (event.source != TASK_ID_BLE)
            {
                break;
            }
            app_manager_start_driving();
            break;
        case EVENT_REQUEST_STOP:
            if (event.source != TASK_ID_BLE)
            {
                break;
            }
            app_manager_stop_driving();
            break;
        case EVENT_FINISHED:
            if (event.source != TASK_ID_ALGO)
            {
                break;
            }
            break;
        case EVENT_FAIL:
            break;
        }
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
        // ESP_LOGI(TAG, "Yaw: %.1f", algo_euler_angles.yaw * 180.0 / M_PI);
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

void app_manager_algo_task_notify(app_manager_task_flag_type task_flag)
{
    if (algo_running)
    {
        app_manager_task_notify(app_manager_algo_task_handle, task_flag);
    }
}

void app_manager_ble_task_notify(app_manager_task_flag_type task_flag)
{
    if (ble_is_connected())
    {
        app_manager_task_notify(app_manager_ble_main_task_handle, task_flag);
    }
}
