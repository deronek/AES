#pragma once

#ifndef _MPU9255_H
#define _MPU9255_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "esp_err.h"
#include "esp_log.h"
#include "task_utils.h"
#include "app_manager.h"

// constants

// enums

// structs

typedef struct mpu9255_fifo_data_type_tag
{
    union
    {
        struct
        {
            short x, y, z;
        };
        short array[3];
    } gyro_raw;
    union
    {
        struct
        {
            short x, y, z;
        };
        short array[3];
    } accel_raw;
    union
    {
        struct
        {
            long w, x, y, z;
        };
        long array[4];
    } quaternion;
} mpu9255_fifo_data_type;

// global variables
extern QueueHandle_t mpu9255_queue_fifo_data;
// extern SemaphoreHandle_t mpu9255_semaphore_fifo_data_ready;
extern unsigned long timestamp;

// function declarations

esp_err_t mpu9255_init();
TASK mpu9255_task_measure();

#endif