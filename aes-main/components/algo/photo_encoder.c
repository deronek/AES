#include "photo_encoder.c"

#include <math.h>

#include "driver/gpio.h"

#define PHOTO_ENCODER_GPIO_PIN_L GPIO_NUM_4
#define PHOTO_ENCODER_GPIO_PIN_R GPIO_NUM_2

// number of stripes on the wheel encoder
#define N 20

// diameter of wheel in micrometers
#define D (59 * 1000)

// radius of wheel
#define R (D / 2)

// width of a stripe on the wheel encoder in micrometers
#define WIDTH_STRIPE (3862)

// distance between two stripes on the wheel encoder in micrometers
#define DISTANCE_STRIPE (5408)

// // distance of full wheel turn
// #define DIST_FULL_TURN (2 * M_PI * R)

// local variables

// distance in micrometers
static uint32_t distance_l = 0;
static uint32_t distance_r = 0;

/**
 * @todo Implement some filtering using this interrupt
 */
// local function declarations

static void IRAM_ATTR photo_encoder_l_isr(void *arg)
{
    int level = gpio_get_level(PHOTO_ENCODER_GPIO_PIN_L);

    if (level)
    {
        distance_l += WIDTH_STRIPE;
    }
    else
    {
        distance_l += DISTANCE_STRIPE;
    }
}

static void IRAM_ATTR photo_encoder_r_isr(void *arg)
{
    int level = gpio_get_level(PHOTO_ENCODER_GPIO_PIN_R);

    if (level)
    {
        distance_r += WIDTH_STRIPE;
    }
    else
    {
        distance_r += DISTANCE_STRIPE;
    }
}

void photo_encoder_init()
{
    gpio_reset_pin(PHOTO_ENCODER_GPIO_PIN_L);
    gpio_set_direction(PHOTO_ENCODER_GPIO_PIN_L, GPIO_MODE_INPUT);

    gpio_reset_pin(PHOTO_ENCODER_GPIO_PIN_R);
    gpio_set_direction(PHOTO_ENCODER_GPIO_PIN_R, GPIO_MODE_INPUT);

    /**
     * @brief Count both edges. Arbitration is done in ISR.
     */
    gpio_set_intr_type(PHOTO_ENCODER_GPIO_PIN_L, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(PHOTO_ENCODER_GPIO_PIN_L, photo_encoder_l_isr, NULL);

    gpio_set_intr_type(PHOTO_ENCODER_GPIO_PIN_R, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(PHOTO_ENCODER_GPIO_PIN_R, photo_encoder_r_isr, NULL);
}

uint32_t photo_encoder_get_distance()
{
    return (distance_l + distance_r) / 2;
}
