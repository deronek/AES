#pragma once

#ifndef _POSITION_H
#define _POSITION_H

#include "task_utils.h"

// constants

// #define POSITION_RECTANGLE
#define POSITION_TRAPEZOIDAL

// structs
typedef struct algo_position_type_tag
{
    float x;
    float y;
} algo_position_type;

// global variables
extern algo_position_type algo_position;

// function declarations
void position_init();
void position_reset();
void position_calculate();

TASK position_photo_encoder_process();
void position_photo_encoder_process_request_stop();

TASK position_accel_process();
void position_accel_process_request_stop();

#endif