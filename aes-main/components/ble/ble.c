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
    }
}

esp_err_t ble_send_data(void *data, uint8_t len)
{

}
