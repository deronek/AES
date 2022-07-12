#pragma once

#ifndef _GOAL_HEADING_H
#define _GOAL_HEADING_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mpu9255.h"
#include "esp_err.h"
#include "esp_log.h"
#include "task_utils.h"
#include "hc_sr04.h"

// constants

/**
 * @brief Refactor all of below data as parameters (maybe passed via BLE).
 * Assume the (0, 0) point is the bottom right point and coordinates
 * can be only positive.
 */
// border measured manually
#define X_BORDER (3.05)
#define Y_BORDER (6.1)

#define X_START (0.455)
#define Y_START (0.0)

#define X_FINISH (3.05)
#define Y_FINISH (5.7)

// enums

// structs

// global variables
extern float algo_goal_heading;

// function declarations
void goal_heading_init();
void goal_heading_calculate();
float goal_heading_angle_to_goal();
float goal_heading_distance_to_goal();

#endif