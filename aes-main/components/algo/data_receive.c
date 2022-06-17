#include "data_receive.h"

// global variables
mpu9255_quaternion_data_type algo_mpu9255_quaternion_data;
// photo_encoder_position_type algo_photo_encoder_position;
hc_sr04_data_type algo_hc_sr04_data;

// local variables

// function definitions

void algo_data_receive_init()
{
}

void algo_data_receive_get_latest()
{
    xQueuePeek(mpu9255_queue_quaternion_data, &algo_mpu9255_quaternion_data, 0);
    // xQueuePeek(hc_sr04_queue_data, &algo_hc_sr04_data, 0);
}