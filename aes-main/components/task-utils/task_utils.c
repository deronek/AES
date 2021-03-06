#include "task_utils.h"

// global variables

// local variables

static const char *TAG = "task_utils";

// function declarations

void task_utils_create_task(TaskFunction_t task,
                            const char *task_name,
                            configSTACK_DEPTH_TYPE stack_depth,
                            void *parameters,
                            UBaseType_t priority,
                            TaskHandle_t *handle,
                            BaseType_t core_id)
{
    BaseType_t retval = xTaskCreatePinnedToCore(
        task,
        task_name,
        stack_depth, // TODO: change stack
        parameters,
        priority,
        handle,
        core_id);

    if (retval != pdPASS)
    {
        ESP_LOGE(TAG, "Task %s creation unsuccessful", task_name);
        abort();
    }

    /**
     * @todo If we log anything during initialization (or block initialization task
     * in any way, like queue poll), initialization task actually gives CPU time
     * to the application tasks.
     *
     * Either minimize logging and blocking in initialization task, or make 100 % sure that
     * giving application tasks CPU time during initialization won't result in
     * undefined behaviour.
     */
    // ESP_LOGI(TAG, "Task %s created successfully, stack depth %hu, priority %u, core %d",
    //          task_name, stack_depth, priority, core_id);
}

void task_utils_request_delete_task(TaskHandle_t *task_handle, void (*request_stop_fun)())
{
    ESP_LOGI(TAG, "Requested stop");
    if (request_stop_fun != NULL)
    {
        request_stop_fun();
    }
    /**
     * @brief Wait for the notification from the task that
     * that it is ready to be deleted.
     */
    ESP_LOGI(TAG, "Waiting for notification");
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    ESP_LOGI(TAG, "Deleting task");
    vTaskDelete(*task_handle);
    *task_handle = NULL;
}