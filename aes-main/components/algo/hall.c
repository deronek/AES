#include "hall.h"

#include "driver/gpio.h"

#define HALL_GPIO_PIN GPIO_NUM_15

// local variables
static bool detected = false;

/**
 * @todo Implement some filtering using this interrupt
 */
// local function declarations
static void IRAM_ATTR hall_isr(void *arg)
{
    detected = true;
}

void hall_init()
{
    gpio_reset_pin(HALL_GPIO_PIN);
    gpio_set_direction(HALL_GPIO_PIN, GPIO_MODE_INPUT);
    gpio_set_intr_type(HALL_GPIO_PIN, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(HALL_GPIO_PIN, hall_isr, NULL);
}

bool hall_get_detected()
{
    return detected;
}