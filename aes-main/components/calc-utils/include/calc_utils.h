#pragma once

#ifndef _CALC_UTILS
#define _CALC_UTILS

#include "app_manager.h"
#include "esp_err.h"
#include "esp_log.h"
#include "task_utils.h"

// constants

// enums

// structs

// global variables

// function declarations
float calc_utils_interpolate(const float x, const float y, const uint8_t num_values, const float value);

// to get quaternion, scale down by 30 bit

#endif