#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "us.h"
#include "i2c.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu9255.h"
#include "app_manager.h"

static const char *TAG = "main";

#define APP_MANAGER_INIT_STACK_DEPTH 4096
#define APP_MANAGER_INIT_PRIORITY 20

void app_main(void)
{
    /*
        Increase priority so we initialize everything
        without other tasks taking the CPU time.
        This task will exit on its own after initialization.
    */
    task_utils_create_task(app_manager_init,
                           "app_manager_init",
                           APP_MANAGER_INIT_STACK_DEPTH,
                           NULL,
                           APP_MANAGER_INIT_PRIORITY,
                           NULL,
                           0);
}