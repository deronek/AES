#pragma once

#ifndef _MOTOR_H
#define _MOTOR_H

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/rmt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "xtensa/hal.h"

#include "app_manager.h"
#include "task_utils.h"

// constants

// enums

// structs
/**
 * @todo Might refactor this structure,
 * depending of the outputs of algo module
 *
 */
typedef struct motor_control_input_data_type_tag
{
    float speed;
    float angle;
} motor_control_input_data_type;

// global variables

// data queue, will only be read by queue peek

// function declarations

void motor_init();
TASK motor_main();

#endif