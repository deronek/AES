#include "data_receive.h"

// global variables
mpu9255_quaternion_data_type algo_mpu9255_quaternion_data;
algo_position_type algo_position;
// photo_encoder_position_type algo_photo_encoder_position;
hc_sr04_data_type algo_hc_sr04_data;

// local variables
static const char *TAG = "algo-data-receive";

// function definitions

void algo_data_receive_init()
{
}

void algo_data_receive_get_latest()
{
    BaseType_t retval;
    retval = xQueuePeek(mpu9255_queue_quaternion_data, &algo_mpu9255_quaternion_data, 0);
    if (retval != pdPASS)
    {
        ESP_LOGE(TAG, "mpu9255_queue_quaternion_data empty");
    }
    retval = xQueuePeek(algo_position_queue, &algo_position, 0);
    if (retval != pdPASS)
    {
        ESP_LOGE(TAG, "algo_position_queue empty");
    }
    // xQueuePeek(hc_sr04_queue_data, &algo_hc_sr04_data, 0);
}