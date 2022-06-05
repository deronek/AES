#include "photo_encoder.h"

#include <math.h>

#include "esp_attr.h"
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

// global variables
photo_encoder_distance_type photo_encoder_distance;

// local variables

/**
 * @todo Implement some filtering using this interrupt
 */
// local function definitions
static void IRAM_ATTR photo_encoder_l_isr(void *arg);
static void IRAM_ATTR photo_encoder_r_isr(void *arg);

// local function declarations

static void IRAM_ATTR photo_encoder_l_isr(void *arg)
{
    int level = gpio_get_level(PHOTO_ENCODER_GPIO_PIN_L);

    if (level)
    {
        photo_encoder_distance.left += WIDTH_STRIPE;
    }
    else
    {
        photo_encoder_distance.left += DISTANCE_STRIPE;
    }
}

static void IRAM_ATTR photo_encoder_r_isr(void *arg)
{
    int level = gpio_get_level(PHOTO_ENCODER_GPIO_PIN_R);

    if (level)
    {
        photo_encoder_distance.right += WIDTH_STRIPE;
    }
    else
    {
        photo_encoder_distance.right += DISTANCE_STRIPE;
    }
}

// function declarations

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
    gpio_set_intr_type(PHOTO_ENCODER_GPIO_PIN_R, GPIO_INTR_ANYEDGE);

    /**
     * @brief Initialize output data struct.
     */
    photo_encoder_distance.left = 0;
    photo_encoder_distance.right = 0;
}

void photo_encoder_enable_isr()
{
    gpio_isr_handler_add(PHOTO_ENCODER_GPIO_PIN_L, photo_encoder_l_isr, NULL);
    gpio_isr_handler_add(PHOTO_ENCODER_GPIO_PIN_R, photo_encoder_r_isr, NULL);
}

void photo_encoder_disable_isr()
{
    gpio_isr_handler_remove(PHOTO_ENCODER_GPIO_PIN_L);
    gpio_isr_handler_remove(PHOTO_ENCODER_GPIO_PIN_R);
}
