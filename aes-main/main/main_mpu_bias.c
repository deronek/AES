#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "us.h"
#include "i2c.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu9255.h"

static const char *TAG = "main";


void app_main(void)
{
    i2c_master_init();
    mpu9255_init();
    // mpu_reset_fifo();
    vTaskDelay(pdMS_TO_TICKS(100));

    BaseType_t retval;
    TaskHandle_t handle;
    retval = xTaskCreate(
        mpu9255_task_measure,
        "mpu9255_task_measure",
        2048, // TODO: change stack
        NULL,
        2,
        &handle
    );

    if(retval != pdPASS)
    {
        ESP_LOGE(TAG, "Task creation unsuccessful");
        abort();
    }


    // long accel_bias[3];
    // ESP_ERROR_CHECK(mpu_read_6500_accel_bias(accel_bias));
    // printf("Accel bias: %lx %lx %lx\n", accel_bias[0], accel_bias[1], accel_bias[2]);

    // long gyro_bias[3];
    // ESP_ERROR_CHECK(mpu_read_6500_gyro_bias(gyro_bias));
    // printf("Gyro bias: %lx %lx %lx\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}