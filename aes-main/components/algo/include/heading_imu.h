#pragma once

#ifndef _HEADING_IMU_H
#define _HEADING_IMU_H

// structs

typedef struct algo_heading_data_type_tag
{
    float heading;
} algo_heading_data_type;

// global variables
extern algo_heading_data_type algo_heading;

// function declarations
void heading_imu_init();
void heading_imu_calculate();
void heading_imu_reset();

#endif