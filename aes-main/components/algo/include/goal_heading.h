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