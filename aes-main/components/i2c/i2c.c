#include "i2c.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "app_manager.h"

// global variables
SemaphoreHandle_t i2c_mutex_handle;

// local variables

static const char *TAG = "i2c";

// function declarations

// function definitions

/**
 * @brief i2c master initialization
 */
void i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

    i2c_mutex_handle = xSemaphoreCreateBinary();

    if (i2c_mutex_handle == NULL)
    {
        abort();
    }
    xSemaphoreGive(i2c_mutex_handle);

    ESP_LOGI(TAG, "I2C initialized successfully");

    xTaskNotifyGive(app_manager_init_task_handle);
    vTaskSuspend(NULL);
}