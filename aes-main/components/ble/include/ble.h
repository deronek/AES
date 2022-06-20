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
#define BLE_NOTIFY_TX_TIMEOUT (pdMS_TO_TICKS(100))

// enums

// structs

/**
 * @brief Task ID used for BLE frame distinction
 */
typedef enum ble_task_id_type_tag
{
    TASK_ID_HC_SR04 = 0,
    TASK_ID_MPU9255 = 1,
    TASK_ID_ALGO = 2,
    TASK_ID_BLE = 3,
    TASK_ID_APP_MANAGER = 4,
} ble_task_id_type;
typedef struct ble_notify_tx_type_tag
{
    ble_task_id_type source;
    void *data;
} ble_notify_tx_type;

// global variables
extern bool ble_running;
extern QueueHandle_t ble_notify_tx_queue;

// function declarations
void ble_init();
bool ble_is_connected();
TASK ble_notify_main();
TASK ble_heartbeat();
void ble_send_from_task(ble_task_id_type source, void *data);

#endif