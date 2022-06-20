#pragma once

#ifndef _APP_MANAGER_H
#define _APP_MANAGER_H

#include "esp_err.h"

#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc.h"

#include "task_utils.h"
#include "i2c.h"
#include "mpu9255.h"
#include "algo.h"
#include "hc_sr04.h"
#include "ble.h"

// constants
// #define configTASK_NOTIFICATION_ARRAY_ENTRIES 2

// enums
typedef enum app_manager_state_type_tag
{
    APP_MANAGER_INIT,
    APP_MANAGER_READY,
    APP_MANAGER_CALIBRATING,
    APP_MANAGER_DRIVING,
    APP_MANAGER_FINISHED,
    APP_MANAGER_FAIL
} app_manager_state_type;

// typedef enum app_manager_task_flag_type_tag
// {
//     TASK_FLAG_HC_SR04 = (1 << TASK_ID_HC_SR04),
//     TASK_FLAG_MPU9255 = (1 << TASK_ID_MPU9255),
//     TASK_FLAG_ALGO = (1 << TASK_ID_ALGO)
// } app_manager_task_flag_type;

typedef struct app_manager_ble_data_type_tag
{
    app_manager_state_type state;
} app_manager_ble_data_type;

typedef enum app_manager_event_id_type_tag
{
    EVENT_REQUEST_START,
    EVENT_REQUEST_STOP,
    EVENT_STARTED_DRIVING,
    EVENT_FINISHED_DRIVING,
    EVENT_FAIL,
} app_manager_event_id_type;

// structs

typedef struct app_manager_event_type_tag
{
    ble_task_id_type source;
    app_manager_event_id_type type;
} app_manager_event_type;

// global variables
extern TaskHandle_t app_manager_algo_task_handle,
    app_manager_mpu9255_task_handle,
    app_manager_main_task_handle,
    app_manager_hc_sr04_task_handle,
    app_manager_ble_task_handle,
    app_manager_init_task_handle;

extern app_manager_state_type app_manager_state;
extern QueueHandle_t app_manager_event_queue;

extern const uint8_t app_manager_task_data_size[];

// function declarations

TASK app_manager_init();
TASK app_manager_main();
void app_manager_start_driving();
void app_manager_notify_main(ble_task_id_type source, app_manager_event_id_type event_id);
// void app_manager_algo_task_notify(app_manager_task_flag_type task_flag);
// void app_manager_ble_task_notify(app_manager_task_flag_type task_flag);

#endif