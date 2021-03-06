#pragma once

#ifndef _BORDER_RECOIL_H
#define _BORDER_RECOIL_H

// enums
typedef enum border_recoil_state_type_tag
{
    BORDER_RECOIL_NONE,
    BORDER_RECOIL_DIRECTION_LEFT,
    BORDER_RECOIL_DIRECTION_RIGHT
} border_recoil_state_type;

// global variables
extern border_recoil_state_type border_recoil_state;
extern float border_recoil_coefficient;

// function declarations
void border_recoil_init();
void border_recoil_calculate();
void border_recoil_reset();
float border_recoil_get_coefficient_scaled();

#endif