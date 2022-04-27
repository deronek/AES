#include "task_utils.h"
#include "ble.h"
#include "ble_data_types.h"

// global variables

// local variables

static const char *TAG = "ble";

// function defintions
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
    
        xQueuePeek(mpu9255_queue_fifo_data, &malloc_mpu9255_fifo_data, 0);
        xQueuePeek(hc_sr04_queue_data, &algo_hc_sr04_data, 0);

    }
}

esp_err_t ble_send_data(void *data, uint8_t len)
{
    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], sizes[i], string[i], false); // change sizes[i] and string[i] 
}
