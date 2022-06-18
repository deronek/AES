#pragma once

#ifndef _DESIRED_HEADING_H
#define _DESIRED_HEADING_H

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
extern float algo_desired_heading;

// function declarations
void desired_heading_init();
void desired_heading_calculate();

#endif