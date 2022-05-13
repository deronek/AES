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

// enums

// structs
typedef struct algo_quaternion_type_tag
{
    float w;
    float x;
    float y;
    float z;
} algo_quaternion_type;

typedef struct algo_euler_angles_type_tag
{
    float pitch;
    float yaw;
    float roll;
} algo_euler_angles_type;

typedef struct algo_heading_data_type_tag
{
    int16_t heading;
} algo_heading_data_type;

// global variables
extern algo_quaternion_type algo_quaternion;
extern algo_euler_angles_type algo_euler_angles;
extern QueueHandle_t algo_heading_data_queue;
extern bool algo_running;

// function declarations
void algo_init();
TASK algo_main();
void algo_run();

// to get quaternion, scale down by 30 bit

#endif