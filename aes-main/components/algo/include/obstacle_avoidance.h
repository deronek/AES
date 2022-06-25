#pragma once

#ifndef _OBSTACLE_AVOIDANCE_H
#define _OBSTACLE_AVOIDANCE_H

#include "freertos/FreeRTOS.h"

// enums
typedef enum obstacle_avoidance_state_type_tag
{
    OA_BEHAVIOUR_NONE,
    OA_BEHAVIOUR_FOLLOW_WALL_CLOCKWISE,
    OA_BEHAVIOUR_FOLLOW_WALL_COUNTERCLOCKWISE,
    OA_BEHAVIOUR_AVOID_OBSTACLE
} obstacle_avoidance_state_type;

// constants
#define DANGER_LEVELS_NUMBER 6
#define DANGER_LEVEL_MAX 5
#define DANGER_LEVEL_MIN 0

// global variables
extern uint8_t algo_obstacle_avoidance_heading_sector;
extern uint8_t algo_obstacle_avoidance_danger_level_in_heading;
extern float algo_follow_wall_angle;
extern bool algo_obstacle_avoidance_request_follow_wall;
extern obstacle_avoidance_state_type algo_obstacle_avoidance_state;
extern float algo_avoid_obstacle_angle;

// function declarations;
void obstacle_avoidance_init();
void obstacle_avoidance_calculate();
void obstacle_avoidance_reset();
void obstacle_avoidance_force_cw();
void obstacle_avoidance_force_ccw();

#endif