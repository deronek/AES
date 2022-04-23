#pragma once

#ifndef _BLE_DATA_TYPES_H
#define _BLE_DATA_TYPES_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"


// constants
typedef struct ble_hc_sr04_data_type_tag {
     uint8_t id;
     hc_sr04_data_type data;
     uint8_t checksum
} ble_hc_sr04_data_type;

typedef struct ble_mpu92554_data_type_tag {
     uint8_t id;
     mpu9255_fifo_data_type data;
     uint8_t checksum
} ble_mpu9255_data_type;

// enums

// structs

// global variables

// function declarations

#endif