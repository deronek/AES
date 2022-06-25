#include "hall.h"

#include "esp_attr.h"
#include "driver/gpio.h"
#include "driver/adc.h"

#include <stdlib.h>

#define HALL_GPIO_PIN GPIO_NUM_15

/**
 * @todo Tweak this threshold
 */
#define HALL_THRESHOLD (100)

// global variables
bool algo_hall_detected = false;

/**
 * @todo Implement some filtering using this interrupt
 */
// local function declarations
// static void IRAM_ATTR hall_isr(void *arg)
// {
//     algo_hall_detected = true;
// }

void hall_init()
{
    gpio_reset_pin(HALL_GPIO_PIN);
    gpio_set_direction(HALL_GPIO_PIN, GPIO_MODE_INPUT);

    /**
     * @todo Tweak attenuation below.
     */
    adc2_config_channel_atten(ADC2_CHANNEL_3, ADC_ATTEN_DB_0);
}

void hall_measure()
{
    int value;
    adc2_get_raw(ADC2_CHANNEL_3, ADC_WIDTH_BIT_12, &value);
    // ESP_LOGI(TAG, "Hall: %d", );

    if (value > HALL_THRESHOLD)
    {
        algo_hall_detected = true;
    }
}

void hall_reset()
{
    algo_hall_detected = false;
}

// void hall_enable_isr()
// {
//     gpio_isr_handler_add(HALL_GPIO_PIN, hall_isr, NULL);
// }

// void hall_disable_isr()
// {
//     gpio_isr_handler_remove(HALL_GPIO_PIN);
// }