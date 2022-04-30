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

#include "mpu9255.h"
#include "hc_sr04.h"

// constants

// enums

// structs

// global variables

// function declarations
void ble_init();
TASK ble_main();

#endif