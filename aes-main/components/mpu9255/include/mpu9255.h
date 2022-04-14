#pragma once

#ifndef _MPU9255_H
#define _MPU9255_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "esp_err.h"
#include "esp_log.h"
#include "task_utils.h"

// constants



// enums


// structs





// global variables
extern short gyro_raw[3], accel_raw[3];
extern long quat[4];
extern unsigned long timestamp;

// function declarations

esp_err_t mpu9255_init();
void mpu9255_task_measure();

#endif