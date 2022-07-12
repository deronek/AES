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
} motor_control_input_data_type;

typedef struct motor_control_output_data_type_tag
{
    uint8_t dir1;
    uint8_t dir2;
    float pwm1;
    float pwm2;
} motor_control_output_data_type;

// global variables
extern motor_control_output_data_type motor_control_output_data;

// data queue, will only be read by queue peek

// function declarations

void motor_init();
void motor_start(float goal_heading);
void motor_tick();
void motor_reset();
void motor_run_tc();
bool motor_is_turning();

#endif