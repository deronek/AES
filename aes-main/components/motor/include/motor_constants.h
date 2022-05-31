#pragma once

#ifndef _MOTOR_CONSTANTS_H
#define _MOTOR_CONSTANTS_H

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/rmt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "xtensa/hal.h"

// constants

#define MOTOR_NUM_VALUES_HEADING_TURN 5

// enums

// structs

// global variables
extern float motor_heading_turn_x[MOTOR_NUM_VALUES_HEADING_TURN];
extern float motor_heading_turn_y[MOTOR_NUM_VALUES_HEADING_TURN];

// data queue, will only be read by queue peek

// function declarations

#endif