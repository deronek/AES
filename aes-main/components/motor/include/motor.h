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
    float current_heading;
    float desired_heading;
    bool request_critical_steering;
} motor_control_input_data_type;

// global variables

// data queue, will only be read by queue peek

// function declarations

void motor_init();
void motor_start(float goal_heading);
void motor_tick();
void motor_reset();

#endif