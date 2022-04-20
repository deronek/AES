#pragma once

#ifndef _ALGO_H
#define _ALGO_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

#include "mpu9255.h"
#include "esp_err.h"
#include "esp_log.h"
#include "task_utils.h"

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

// global variables
extern algo_quaternion_type algo_quaternion;
extern algo_euler_angles_type algo_euler_angles;

// function declarations
TASK algo_main();
void algo_run();

// to get quaternion, scale down by 30 bit

#endif