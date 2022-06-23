#pragma once

#ifndef _REFLECTANCE_H
#define _REFLECTANCE_H

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "xtensa/hal.h"

#include "task_utils.h"

// constants

// enums
typedef enum reflectance_request_avoidance_type_tag
{
    NO_REQUEST_AVOIDANCE,
    REQUEST_AVOIDANCE
} reflectance_request_avoidance_type;

// structs
typedef struct reflectance_request_avoidance_data_type_tag
{
    reflectance_request_avoidance_type left;
    reflectance_request_avoidance_type right;
} reflectance_request_avoidance_data_type;

// global variables
extern reflectance_request_avoidance_data_type reflectance_request_avoidance_data;

// function declarations

void reflectance_init();
TASK reflectance_main();

#endif