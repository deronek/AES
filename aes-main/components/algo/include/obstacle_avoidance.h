#pragma once

#ifndef _OBSTACLE_AVOIDANCE_H
#define _OBSTACLE_AVOIDANCE_H

#include "freertos/FreeRTOS.h"

// enums
typedef enum obstacle_avoidance_state_type_tag
{
    OBSTACLE_AVOIDANCE_NONE,
    OBSTACLE_AVOIDANCE_CLOCKWISE,
    OBSTACLE_AVOIDANCE_COUNTERCLOCKWISE
} obstacle_avoidance_state_type;

// constants
#define DANGER_LEVELS_NUMBER 6
#define DANGER_LEVEL_MAX 5
#define DANGER_LEVEL_MIN 0

// global variables
extern float algo_obstacle_avoidance_angle;
extern uint8_t algo_obstacle_avoidance_heading_sector;
extern uint8_t algo_obstacle_avoidance_danger_level_in_heading;
extern float algo_follow_wall_angle;
extern bool algo_obstacle_avoidance_request_follow_wall;
extern obstacle_avoidance_state_type algo_obstacle_avoidance_state;

// function declarations;
void obstacle_avoidance_init();
void obstacle_avoidance_calculate();
void obstacle_avoidance_reset();

#endif