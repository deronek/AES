#include "tasks.h"

const TaskFunction_t tasks[TASK_NUMBER] = {
    mpu9255_task_measure,
    app_manager_main,
};

const char *task_names[TASK_NUMBER] =
    {
        "mpu9255_task_measure",
        "app_manager_main"

};
const BaseType_t task_priorities[TASK_NUMBER] = {
    3,
    2};

// TODO: change this to some smaller value, can test how much stack is actually used
const configSTACK_DEPTH_TYPE task_stack_sizes[TASK_NUMBER] = {
    2048,
    2048};

const BaseType_t task_core_ids[TASK_NUMBER] = {
    0,
    0};