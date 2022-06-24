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

// structs
typedef struct reflectance_request_avoidance_type_tag
{
    bool left;
    bool right;
} reflectance_request_avoidance_type;

// global variables
extern reflectance_request_avoidance_type reflectance_request_avoidance;

// function declarations

void reflectance_init();
TASK reflectance_main();
void reflectance_clear_left_request();
void reflectance_clear_right_request();

#endif