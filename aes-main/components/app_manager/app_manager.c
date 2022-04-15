#include "app_manager.h"

// constants
#define TASK_TICK_PERIOD pdMS_TO_TICKS(10)

// global variables
app_manager_state_type app_manager_state;

// local variables
static const char *TAG = "app_manager";

/**
 * @brief Task handles used in the app manager.
 */
static TaskHandle_t task_handles[TASK_NUMBER];

// function declarations
static void app_manager_run();
static void app_manager_update_state();
static void app_manager_init_peripherals();
static void app_manager_create_tasks();
inline static void log_abort_wrong_state(const char *state);

// function definitions
inline static void log_abort_wrong_state(const char *state)
{
    ESP_LOGE(TAG, "app_manager_state == %s in main loop", state);
    abort();
}

void app_manager_init()
{
    app_manager_state = APP_MANAGER_INIT;

    app_manager_init_peripherals();
    app_manager_create_tasks();

    // estabilish communication with user here

    app_manager_state = APP_MANAGER_READY;

    ESP_LOGI(TAG, "App manager initialization successful");
    vTaskDelete(NULL);
    abort();
}

void app_manager_init_peripherals()
{
    i2c_master_init();
    mpu9255_init();
}

void app_manager_create_tasks()
{
    // TODO: implement some scalable interface for creating tasks
    for (uint8_t i = 0; i < TASK_NUMBER; ++i)
    {
        ESP_LOGI(TAG, "Priority is still %d", uxTaskPriorityGet(NULL));
        task_utils_create_task(
            tasks[i],
            task_names[i],
            task_stack_sizes[i],
            NULL,
            task_priorities[i],
            &(task_handles[i]),
            task_core_ids[i]);
    }
}

TASK app_manager_main()
{
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;)
    {
        app_manager_run();
        app_manager_update_state();
        task_utils_sleep_or_warning(&last_wake_time, TASK_TICK_PERIOD, TAG);
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