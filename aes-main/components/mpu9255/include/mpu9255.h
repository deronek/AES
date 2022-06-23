#pragma once

#ifndef _MPU9255_H
#define _MPU9255_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "esp_err.h"
#include "esp_log.h"
#include "task_utils.h"

// constants
#define MPU9255_AXIS_NUMBER 3
#define MPU9255_INT_PIN GPIO_NUM_13

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

typedef union mpu9255_quaternion_type_tag
{
    struct
    {
        long w, x, y, z;
    };
    long array[4];
} mpu9255_quaternion_data_type;

// global variables
extern QueueHandle_t mpu9255_queue_quaternion_data;
extern QueueHandle_t mpu9255_queue_accel_data;
// extern SemaphoreHandle_t mpu9255_semaphore_fifo_data_ready;
extern unsigned long timestamp;

// function declarations

esp_err_t mpu9255_init();
void mpu9255_get_calibration();
esp_err_t mpu9255_apply_calibration();
TASK mpu9255_task_measure();
void mpu9255_reset();
esp_err_t mpu9255_set_isr(bool enabled);
void mpu9255_set_ble_sending(bool enabled);

#endif