#pragma once

#ifndef _HEADING_IMU_H
#define _HEADING_IMU_H

// global variables
extern float heading_imu_current;

// function declarations
void heading_imu_init();
void heading_imu_calculate();

#endif