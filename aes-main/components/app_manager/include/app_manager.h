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
#include "tasks.h"
#include "i2c.h"
#include "mpu9255.h"

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

// structs

// global variables

extern app_manager_state_type app_manager_state;

// function declarations

void app_manager_init();
TASK app_manager_main();

#endif