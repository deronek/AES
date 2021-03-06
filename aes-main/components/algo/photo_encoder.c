#include "photo_encoder.h"

// #include <math.h>

#include "heading_imu.h"
#include "motor.h"

// #include "esp_attr.h"
// #include "app_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
// #include "hal/gpio_types.h"

#define PHOTO_ENCODER_GPIO_PIN_L (GPIO_NUM_2)
#define PHOTO_ENCODER_GPIO_PIN_R (GPIO_NUM_4)

// constants

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
    // if (motor_is_turning())
    // {
    //     return;
    // }
    BaseType_t retval;
    BaseType_t higher_task_woken = pdFALSE;

    photo_encoder_event_type event;
    event.wheel = PHOTO_ENCODER_L;

    /**
     * @brief Read current heading. This read is atomic.
     * Using uint32_t because declaring float in ISR is not supported.
     * We do not perform any operation on this number, just read.
     *
     * @todo It might be possible to use photo encoder data
     * to also get heading change. Maybe somehow use this
     * to make the position measurement more accurate.
     */
    volatile double heading = algo_current_heading;
    event.heading = (float)heading;

    volatile int wheel_direction = motor_control_output_data.dir2;
    event.wheel_direction = wheel_direction;

    retval = xQueueSendFromISR(photo_encoder_event_queue, &event, &higher_task_woken);
    if (retval == errQUEUE_FULL)
    {
        ESP_DRAM_LOGE(TAG, "photo_encoder_event_queue full, tick lost");
    }

    portYIELD_FROM_ISR(higher_task_woken);
}

static void photo_encoder_r_isr(void *arg)
{
    // if (motor_is_turning())
    // {
    //     return;
    // }
    BaseType_t retval;
    BaseType_t higher_task_woken = pdFALSE;

    photo_encoder_event_type event;
    event.wheel = PHOTO_ENCODER_R;

    /**
     * @brief Read current heading. This read is atomic.
     * Using uint32_t because declaring float in ISR is not supported.
     * We do not perform any operation on this number, just read.
     *
     * @todo It might be possible to use photo encoder data
     * to also get heading change. Maybe somehow use this
     * to make the position measurement more accurate.
     */
    volatile double heading = algo_current_heading;
    event.heading = (float)heading;

    volatile int wheel_direction = motor_control_output_data.dir1;
    event.wheel_direction = wheel_direction;

    retval = xQueueSendFromISR(photo_encoder_event_queue, &event, &higher_task_woken);
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
