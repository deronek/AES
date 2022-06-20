#include "photo_encoder.h"

// #include <math.h>

// #include "esp_attr.h"
// #include "app_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
// #include "hal/gpio_types.h"

#define PHOTO_ENCODER_GPIO_PIN_L (GPIO_NUM_4)
#define PHOTO_ENCODER_GPIO_PIN_R (GPIO_NUM_2)

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
QueueHandle_t photo_encoder_event_queue;

// local variables
static const char *TAG = "photo-encoder";
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
    BaseType_t retval;
    BaseType_t higher_task_woken = pdFALSE;
    int level = gpio_get_level(PHOTO_ENCODER_GPIO_PIN_L);
    photo_encoder_event_type event_type;

    if (level)
    {
        event_type = PHOTO_ENCODER_L_WIDTH;
    }
    else
    {
        event_type = PHOTO_ENCODER_L_DISTANCE;
    }

    retval = xQueueSendFromISR(photo_encoder_event_queue, &event_type, &higher_task_woken);
    if (retval == errQUEUE_FULL)
    {
        ESP_DRAM_LOGE(TAG, "photo_encoder_event_queue full, tick lost");
    }

    portYIELD_FROM_ISR(higher_task_woken);
}

static void photo_encoder_r_isr(void *arg)
{
    BaseType_t retval;
    BaseType_t higher_task_woken = pdFALSE;
    int level = gpio_get_level(PHOTO_ENCODER_GPIO_PIN_R);
    photo_encoder_event_type event_type;

    if (level)
    {
        event_type = PHOTO_ENCODER_R_WIDTH;
    }
    else
    {
        event_type = PHOTO_ENCODER_R_DISTANCE;
    }

    retval = xQueueSendFromISR(photo_encoder_event_queue, &event_type, &higher_task_woken);
    if (retval == errQUEUE_FULL)
    {
        ESP_DRAM_LOGE(TAG, "photo_encoder_event_queue full, tick lost");
    }

    portYIELD_FROM_ISR(higher_task_woken);
}

// function declarations

void photo_encoder_init()
{
    int retval;

    /**
     * @brief Initialize notify queue, handled in position_photo_encoder_process.
     */
    photo_encoder_event_queue = xQueueCreate(10, sizeof(photo_encoder_event_type));
    if (photo_encoder_event_queue == NULL)
    {
        abort();
    }

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
