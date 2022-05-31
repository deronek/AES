#pragma once

#ifndef _PHOTO_ENCODER_H
#define _PHOTO_ENCODER_H

#include "esp_err.h"

// structs
typedef struct photo_encoder_distance_type_tag
{
    uint32_t left;
    uint32_t right;
} photo_encoder_distance_type;

// global variables
extern photo_encoder_distance_type photo_encoder_distance;

// function declarations
void photo_encoder_init();
void photo_encoder_enable_isr();
void photo_encoder_disable_isr();

#endif