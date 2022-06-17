#pragma once

#ifndef _POSITION_H
#define _POSITION_H

#include "task_utils.h"

#include "freertos/queue.h"

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
QueueHandle_t position_queue;

// function declarations
void position_init();
void position_reset();

TASK position_process();
void position_process_request_stop();

TASK position_photo_encoder_process();
void position_photo_encoder_process_request_stop();

TASK position_accel_process();
void position_accel_process_request_stop();

#endif