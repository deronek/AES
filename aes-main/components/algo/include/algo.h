#pragma once

#ifndef _ALGO_H
#define _ALGO_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

#include "app_manager.h"
#include "mpu9255.h"
#include "esp_err.h"
#include "esp_log.h"
#include "task_utils.h"

#include "inv_mpu.h"

// constants
/**
 * @todo Tweak this time and make sure that it is respected when calling algo.
 * Maybe even synchronise sensor measurements so they are updated only per 0.1 s.
 */
#define ALGO_FREQUENCY (10)
#define ALGO_DELTA_TIME (1.0 / ALGO_FREQUENCY)

// enums

// structs

// typedef struct algo_euler_angles_type_tag
// {
// float pitch;
// float yaw;
// float roll;
// } algo_euler_angles_type;

typedef struct algo_ble_data_type_tag
{
    float current_heading;
    float pos_x;
    float pos_y;
    float desired_heading;
    uint8_t obstacle_avoidance_heading_sector;
    float final_heading;
} algo_ble_data_type;

// global variables
// extern algo_heading_data_type algo_current_heading;
// extern algo_quaternion_type algo_quaternion;
// extern algo_euler_angles_type algo_euler_angles;
extern TaskHandle_t algo_position_process_task_handle,
    algo_position_photo_encoder_process_task_handle,
    algo_position_accel_process_task_handle;
extern QueueHandle_t algo_ble_data_queue;
extern bool algo_running;

// function declarations
void algo_init();
TASK algo_main();
void algo_run();
void algo_request_stop();

#endif