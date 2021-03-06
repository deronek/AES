#ifndef _ESP_PORT_H
#define _ESP_PORT_H

#ifdef ESP_PLATFORM

#include <math.h>
#include <stdlib.h>

#include "i2c.h"
#include "xtensa/hal.h"
#include "hal/cpu_hal.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "md-driver";

// constants

// use I2C as master
#define I2C_MASTER_NUM 0
#define I2C_MASTER_TIMEOUT_MS 100

#define i2c_write esp_i2c_write
#define i2c_read esp_i2c_read
#define delay_ms esp_delay_ms
#define get_ms esp_get_ms
// labs defined in stdlib.h
// fabsf defined in math.h
#define min(a, b) ((a < b) ? a : b)
#define reg_int_cb esp_reg_int_cb

#define log_e(fmt, ...) ESP_LOGE(TAG, fmt, ##__VA_ARGS__);
#define log_i(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__);
#define log_v(fmt, ...) ESP_LOGV(TAG, fmt, ##__VA_ARGS__);

#define __no_operation() __asm__ __volatile__("nop");

// function definitions

esp_err_t esp_i2c_write(unsigned char device_address, unsigned char reg_addr, unsigned char write_size, unsigned char const *write_buffer);

// inline function declarations

inline esp_err_t esp_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
    xSemaphoreTake(i2c_mutex_handle, portMAX_DELAY);
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, slave_addr, &reg_addr, 1, data, length, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    xSemaphoreGive(i2c_mutex_handle);
    return ret;
}

inline void esp_delay_ms(unsigned long num_ms)
{
    vTaskDelay(pdMS_TO_TICKS(num_ms));
}

inline void esp_get_ms(unsigned long *count)
{
    *count = xTaskGetTickCount();
}

inline void esp_reg_int_cb(struct int_param_s *int_param)
{
    // TODO
    return;
}

#endif

#endif