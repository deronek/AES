#include "photo_encoder.h"

// #include <math.h>

// #include "esp_attr.h"
#include "app_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"

#define PHOTO_ENCODER_GPIO_PIN_L GPIO_NUM_4
#define PHOTO_ENCODER_GPIO_PIN_R GPIO_NUM_2

// constants

// number of stripes on the wheel encoder
#define N (20)

// diameter of wheel in micrometers
#define D (59 * 1000)

// radius of wheel
#define R (D / 2)

// // distance of full wheel turn
// #define DIST_FULL_TURN (2 * M_PI * R)

// structs
// typedef struct photo_encoder_distance_type_tag
// {
// uint32_t left;
// uint32_t right;
// } photo_encoder_distance_type;

// global variables

// local variables
// static QueueHandle_t photo_encoder_queue;
// static photo_encoder_distance_type photo_encoder_distance;

/**
 * @todo Implement some filtering using this interrupt
 */
// local function definitions
static void photo_encoder_l_isr(void *arg);
static void photo_encoder_r_isr(void *arg);

// local function declarations

static void photo_encoder_l_isr(void *arg)
{
    BaseType_t higher_task_woken = pdFALSE;
    int level = gpio_get_level(PHOTO_ENCODER_GPIO_PIN_L);
    photo_encoder_notify_type notify_type;

    if (level)
    {
        notify_type = PHOTO_ENCODER_L_WIDTH;
    }
    else
    {
        notify_type = PHOTO_ENCODER_L_DISTANCE;
    }

    /**
     * @todo Implement some failsafe here that will warn
     * user when we are missing some photo encoder pulses handled.
     * Use xTaskNotifyAndQuery.
     */
    xTaskNotifyFromISR(algo_position_photo_encoder_process_task_handle, notify_type, eSetBits, &higher_task_woken);
    portYIELD_FROM_ISR(higher_task_woken);
}

static void photo_encoder_r_isr(void *arg)
{
    BaseType_t higher_task_woken = pdFALSE;
    int level = gpio_get_level(PHOTO_ENCODER_GPIO_PIN_R);
    photo_encoder_notify_type notify_type;

    if (level)
    {
        notify_type = PHOTO_ENCODER_R_WIDTH;
    }
    else
    {
        notify_type = PHOTO_ENCODER_R_DISTANCE;
    }

    /**
     * @todo Implement some failsafe here that will warn
     * user when we are missing some photo encoder pulses handled.
     * Use xTaskNotifyAndQuery.
     */
    xTaskNotifyFromISR(algo_position_photo_encoder_process_task_handle, notify_type, eSetBits, &higher_task_woken);
    portYIELD_FROM_ISR(higher_task_woken);
}

// function declarations

void photo_encoder_init()
{
    int retval;

    /**
     * @brief Initialize output data struct.
     */
    // photo_encoder_distance.left = 0;
    // photo_encoder_distance.right = 0;

    /**
     * @brief Initialize data queue.
     */
    // photo_encoder_queue = xQueueCreate(1, sizeof(photo_encoder_distance_type));
    // if (photo_encoder_queue == NULL)
    // {
    // abort();
    // }

    gpio_reset_pin(PHOTO_ENCODER_GPIO_PIN_L);
    gpio_set_direction(PHOTO_ENCODER_GPIO_PIN_L, GPIO_MODE_INPUT);

    gpio_reset_pin(PHOTO_ENCODER_GPIO_PIN_R);
    gpio_set_direction(PHOTO_ENCODER_GPIO_PIN_R, GPIO_MODE_INPUT);

    /**
     * @brief Count both edges. Arbitration is done in ISR.
     */
    gpio_set_intr_type(PHOTO_ENCODER_GPIO_PIN_L, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(PHOTO_ENCODER_GPIO_PIN_R, GPIO_INTR_ANYEDGE);
}

// TASK photo_encoder_main()
// {
// for (;;)
// {
//
// xQueueOverwrite(photo_encoder_queue, &photo_encoder_distance);
// }
// }

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
