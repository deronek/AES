#pragma once

#ifndef _I2C_H
#define _I2C_H

#include "esp_err.h"
#include "esp_log.h"

#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc.h"
#include "freertos/semphr.h"

// constants

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0                        /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000               /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

// enums

// structs

// global variables
extern SemaphoreHandle_t i2c_mutex_handle;

// function declarations
esp_err_t i2c_master_init(void);

#endif