#ifndef _TASKS_H
#define _TASKS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mpu9255.h"
#include "app_manager.h"

// constants
#define TASK_NUMBER 2

// global variables
extern const TaskFunction_t tasks[TASK_NUMBER];
extern const char *task_names[TASK_NUMBER];
extern const BaseType_t task_priorities[TASK_NUMBER];
extern const configSTACK_DEPTH_TYPE task_stack_sizes[TASK_NUMBER];
extern const BaseType_t task_core_ids[TASK_NUMBER];

#endif