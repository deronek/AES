#pragma once

#ifndef _BLE_H
#define _BLE_H

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "task_utils.h"

#include "app_manager.h"
#include "mpu9255.h"
#include "hc_sr04.h"

// constants
extern bool ble_running;

// enums

// structs

// global variables
extern TaskHandle_t ble_spp_task_handle;

// function declarations
void ble_init();
bool ble_is_connected();
TASK ble_main();
TASK ble_heartbeat();

#endif