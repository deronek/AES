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
#define MPU9255_AXIS_NUMBER 3
#define MPU9255_INT_PIN GPIO_NUM_19

// enums

// structs
typedef union mpu9255_sensor_data_type_tag
{
    struct
    {
        short x, y, z;
    };
    short array[MPU9255_AXIS_NUMBER];
} mpu9255_sensor_data_type;

typedef struct mpu9255_fifo_data_type_tag
{
    mpu9255_sensor_data_type gyro;
    mpu9255_sensor_data_type accel;
    mpu9255_sensor_data_type mag;
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
void mpu9255_calibrate();
TASK mpu9255_task_measure();

#endif