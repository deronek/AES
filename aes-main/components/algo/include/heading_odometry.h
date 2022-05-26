#pragma once

#ifndef _HEADING_ODOMETRY_H
#define _HEADING_ODOMETRY_H

// global variables
extern float heading_odometry_current;

// function declarations
void heading_odometry_init();
void heading_odometry_calculate();

#endif