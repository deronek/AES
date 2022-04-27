#include "ble.h"

// global variables

// local variables

static const char *TAG = "ble";

// function defintions
void ble_receive_data(uint8_t id, uint8_t* packet);
esp_err_t ble_send_data(void *data, uint8_t len);

// function declarations

void ble_init()
{

}

TASK ble_main()
{
    for(;;)
    {
        /**
         * @brief 
         * - wait for task notification from any sensor task
         * - get data from sensor queue
         * - send that data using ble_send_data()
         */
        /**
         * @todo Implement task notification and getting ID
         */
        uint8_t id = 0;
        uint8_t data_size = app_manager_task_data_size[id];
        uint8_t packet_length = 1 + data_size + 1;

        uint8_t *packet = malloc(packet_length);
        if (packet == NULL)
        {
            ESP_LOGE(TAG, "ble_main malloc");
            abort();
        }

        packet[0] = id;

        ble_receive_data(id, packet);

        /**
         * @todo Implement checksum
         */
        packet[data_size + 1] = 255;
        
        ble_send_data(packet, packet_length);
    }
}


void ble_receive_data(uint8_t id, uint8_t* packet)
{
    switch(id)
    {
        case TASK_ID_HC_SR04:
            xQueuePeek(hc_sr04_queue_data, (packet + 1), 0);
            break;
        case TASK_ID_MPU9255:
            xQueuePeek(mpu9255_queue_fifo_data, (packet + 1), 0);
            break;
        default:
            abort();
    }
}


esp_err_t ble_send_data(void *packet, uint8_t packet_length)
{
    //esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], packet_length, packet, false); // change sizes[i] and string[i] 
    return ESP_OK;
}
