#include "data_receive.h"

// global variables
mpu9255_fifo_data_type algo_mpu9255_fifo_data;
hc_sr04_data_type algo_hc_sr04_data;

// local variables

// function definitions

void algo_data_receive_init()
{
    ;
}

void algo_data_receive_get_latest()
{
    // xQueuePeek(mpu9255_queue_fifo_data, &algo_mpu9255_fifo_data, 0);
    xQueuePeek(hc_sr04_queue_data, &algo_hc_sr04_data, 0);
}