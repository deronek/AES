#pragma once

#ifndef _US_H
#define _US_H

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/rmt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "xtensa/hal.h"

#include "app_manager.h"
#include "task_utils.h"

// constants

#define NUMBER_OF_HC_SR04_SENSORS 2
#define HC_SR04_TIMEOUT pdMS_TO_TICKS(45)

#define HC_SR04_INIT_VALUE 0

// enums
/*
typedef enum hc_sr04_out_status_type_tag
{
    US_OUT_OK,
    US_OUT_TIMEOUT_ECHO_START,
    US_OUT_TIMEOUT_ECHO_END,
    US_OUT_INIT
} hc_sr04_out_status_type;
*/

// structs

/*
typedef struct hc_sr04_sensor_data_type_tag
{
    uint64_t value;
    hc_sr04_out_status_type status;
} hc_sr04_sensor_data_type;
*/
typedef struct hc_sr04_data_type_tag
{
    uint32_t distance[NUMBER_OF_HC_SR04_SENSORS];
} hc_sr04_data_type;
// global variables

// data queue, will only be read by queue peek
extern QueueHandle_t hc_sr04_queue_data;

// function declarations

void hc_sr04_init();
TASK hc_sr04_measure();

#endif