#pragma once

#ifndef _FINAL_HEADING_H
#define _FINAL_HEADING_H

// enums
typedef enum final_heading_behaviour_state_type_tag
{
    BEHAVIOUR_DRIVE_TO_GOAL,
    BEHAVIOUR_FOLLOW_THE_WALL
} final_heading_behaviour_state_type;

// global variables
extern float algo_final_heading;
extern final_heading_behaviour_state_type algo_final_heading_behaviour_state;

// function declarations
void final_heading_init();
void final_heading_calculate();
void final_heading_reset();

#endif