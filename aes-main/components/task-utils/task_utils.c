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
    else
    {
        ESP_LOGI(TAG, "Task %s created successfully, stack depth %hu, priority %u",
                 task_name, stack_depth, priority);
    }
}