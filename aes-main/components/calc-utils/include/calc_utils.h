#pragma once

#ifndef _CALC_UTILS
#define _CALC_UTILS

#include "esp_err.h"
#include "esp_log.h"

// constants
/**
 * @brief Makes sure that angles are in range of [-M_PI, M_PI].
 */
#define ANGLE_SAFEGUARD(x) (atan2f(sinf(x), cosf(x)))

// enums

// structs

// global variables

// function declarations
float calc_utils_interpolate(const float *x, const float *y, const uint8_t num_values, const float value);

// to get quaternion, scale down by 30 bit

#endif