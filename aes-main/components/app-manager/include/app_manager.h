#pragma once

#ifndef _APP_MANAGER_H
#define _APP_MANAGER_H

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/timer.h"

#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc.h"

#include "task_utils.h"
#include "i2c.h"
#include "mpu9255.h"
#include "algo.h"
#include "hc_sr04.h"

// constants

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

typedef enum app_manager_task_id_type_tag
{
    TASK_ID_HC_SR04 = 0,
    TASK_ID_MPU9255 = 1
} app_manager_task_id_type;

// structs

// global variables
extern TaskHandle_t app_manager_algo_task_handle,
    app_manager_mpu9255_task_handle,
    app_manager_main_task_handle,
    app_manager_hc_sr04_task_handle;

extern app_manager_state_type app_manager_state;

extern const uint8_t app_manager_task_data_size[];

// function declarations

TASK app_manager_init();
TASK app_manager_main();

#endif