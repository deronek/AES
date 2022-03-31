#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "us.h"
#include "i2c.h"
#include "inv_mpu.h"

void app_main(void)
{
    i2c_master_init();
    ESP_ERROR_CHECK(mpu_init(NULL));
    ESP_ERROR_CHECK(mpu_set_sensors(INV_XYZ_GYRO));
    short gyro_data[3];
    for (;;)
    {
        ESP_ERROR_CHECK(mpu_get_gyro_reg(gyro_data, NULL));
        printf("X: %hi, Y: %hi, Z: %hi\n", gyro_data[0], gyro_data[1], gyro_data[2]);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}