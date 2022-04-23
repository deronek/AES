#pragma once

#ifndef _BLE_H
#define _BLE_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "task_utils.h"

// constants
typedef struct ble_data_type_tag {
     uint8_t id;
     uint8_t len;
     void *data;
     uint8_t checksum
} ble_data_type;

// enums

// structs

// global variables

// function declarations
void ble_init();
esp_err_t ble_send_data(uint8_t id, uint8_t len, void *data);

#endif