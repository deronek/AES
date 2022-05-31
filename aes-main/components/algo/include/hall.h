#pragma once

#ifndef _HALL_H
#define _HALL_H

#include <stdbool.h>

// global variables
extern bool detected;

// function declarations
void hall_init();
void hall_enable_isr();
void hall_disable_isr();

#endif