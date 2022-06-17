#pragma once

#ifndef _TASK_UTILS_H
#define _TASK_UTILS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

// constants
typedef void TASK;

#define TASK_HZ_TO_TICKS(freq) (1.0f / freq) * configTICK_RATE_HZ

// enums

// structs

// global variables

// function declarations
void task_utils_create_task(TaskFunction_t,
                            const char *,
                            configSTACK_DEPTH_TYPE,
                            void *,
                            UBaseType_t,
                            TaskHandle_t *,
                            BaseType_t);

void task_utils_request_delete_task(TaskHandle_t *task_handle, void (*request_stop_fun)());

// inline function definitions

/**
 * @brief Task utility function to wait until the next expected execution of the task loop.
 *        If task did not slept, this means that its performance is worse than expected by
 *        set execution frequency. This situation logs a warning.
 *
 * @param last_wake_time last tick time that the task woke up
 * @param ticks_to_wait  ticks number between task loop execution
 * @param tag            string identifying the module, used for logging purposes
 */

#define vTaskDelayUntilWorkaround
inline void task_utils_sleep_or_warning(TickType_t *last_wake_time, TickType_t ticks_to_wait, const char *tag)
{
/**
 * @brief vTaskDelayUntil seems bugged, sometimes it does not update the last_wake_time variable, even
 * if the sleep happened.
 * Needed to implement another way to check whether the sleep did not happen.
 */
#ifdef vTaskDelayUntilWorkaround
     TickType_t before_sleep = xTaskGetTickCount();
#endif
     vTaskDelayUntil(last_wake_time, ticks_to_wait);
#ifdef vTaskDelayUntilWorkaround
     *last_wake_time = xTaskGetTickCount();
     if (*last_wake_time == before_sleep)
     {
          ESP_LOGW(tag, "Task did not slept; probable performance issue");
     }
#else
     if (*last_wake_time != xTaskGetTickCount())
     {
          ESP_LOGW(tag, "Task did not slept; probable performance issue");
     }
#endif
}

#endif