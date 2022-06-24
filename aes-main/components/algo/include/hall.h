#pragma once

#ifndef _HALL_H
#define _HALL_H

#include <stdbool.h>

// global variables
extern bool algo_hall_detected;

// function declarations
void hall_init();
void hall_measure();
void hall_reset();

#endif