#pragma once

#ifndef _BLE_H
#define _BLE_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "task_utils.h"

// constants

// enums

// structs

// global variables

// function declarations
void ble_init();
TASK ble_main();

#endif