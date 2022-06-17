#pragma once

#ifndef _DATA_RECEIVE_H
#define _DATA_RECEIVE_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mpu9255.h"
#include "esp_err.h"
#include "esp_log.h"
#include "task_utils.h"
#include "hc_sr04.h"

// constants

// enums

// structs

// global variables

/**
 * @brief This is internal algorithm data structures,
 * only set by function algo_data_receive_get_latest.
 *
 * Do not include this header file in any of the
 * application code except algo features.
 */
extern mpu9255_quaternion_data_type algo_mpu9255_quaternion_data;
// extern photo_encoder_position_type algo_photo_encoder_position;
extern hc_sr04_data_type algo_hc_sr04_data;

// function declarations
void algo_data_receive_get_latest();

// to get quaternion, scale down by 30 bit

#endif