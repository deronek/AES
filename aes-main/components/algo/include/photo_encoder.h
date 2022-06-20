#pragma once

#ifndef _PHOTO_ENCODER_H
#define _PHOTO_ENCODER_H

// #include "esp_err.h"
// #include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// constants

// width of a stripe on the wheel encoder in micrometers
#define WIDTH_STRIPE_UM (3862)
#define WIDTH_STRIPE_M (WIDTH_STRIPE_UM / 1000000.0)

// distance between two stripes on the wheel encoder in micrometers
#define DISTANCE_STRIPE_UM (5408)
#define DISTANCE_STRIPE_M (DISTANCE_STRIPE_UM / 1000000.0)

// enums
typedef enum photo_encoder_event_type_tag
{
    PHOTO_ENCODER_L_WIDTH,
    PHOTO_ENCODER_L_DISTANCE,
    PHOTO_ENCODER_R_WIDTH,
    PHOTO_ENCODER_R_DISTANCE,
} photo_encoder_event_type;

// structs
typedef struct photo_encoder_position_type_tag
{
    float x;
    float y;
} photo_encoder_position_type;

// global variables
extern QueueHandle_t photo_encoder_event_queue;
// extern QueueHandle_t photo_encoder_queue;
// extern photo_encoder_distance_type photo_encoder_distance;

// function declarations
void photo_encoder_init();
void photo_encoder_enable_isr();
void photo_encoder_disable_isr();

#endif