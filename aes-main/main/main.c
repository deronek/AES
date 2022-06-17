#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_manager.h"

static const char *TAG = "main";

#define APP_MANAGER_INIT_STACK_DEPTH 4096
#define APP_MANAGER_INIT_PRIORITY 10

void app_main(void)
{
    /**
     * @brief Wait a little bit to make sure that
     * CORE 1 moves to IDLE.
     */
    vTaskDelay(10);

    /**
     * @brief Create initialization task, which will
     * initialize every module and run the app manager.
     */
    task_utils_create_task(app_manager_init,
                           "app_manager_init",
                           APP_MANAGER_INIT_STACK_DEPTH,
                           NULL,
                           APP_MANAGER_INIT_PRIORITY,
                           &app_manager_init_task_handle,
                           0);
}