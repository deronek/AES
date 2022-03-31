#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "us.h"
#include "i2c.h"
#include "inv_mpu.h"

void app_main(void)
{
    us_init();
    for(;;)
    {
        us_measure();
        us_out_type out = us_out[0];
        switch(out.status)
        {
            case US_OUT_OK:
                printf("distance: %llu\n", out.value);
                break;
            case US_OUT_TIMEOUT_ECHO_START:
                printf("timeout at start echo\n");
                break;
            case US_OUT_TIMEOUT_ECHO_END:
                printf("timeout at end echo\n");
                break;
            default:
                printf("measurement not ready\n");
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}