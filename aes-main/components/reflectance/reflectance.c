#include "reflectance.h"
#include "driver/adc.h"
#include "driver/gpio.h"

// constants

#define NUMBER_OF_REFLECTANCE_SENSORS (4)

#define REFLECTANCE_SENSOR_MEASUREMENT_PREWAIT (pdMS_TO_TICKS(5))
#define REFLECTANCE_SENSOR_MEASUREMENT_WAIT (pdMS_TO_TICKS(50))

/**
 * @todo Fill in the value.
 */
#define REFLECTANCE_THRESHOLD (0)

// structs

// global variables
reflectance_request_avoidance_data_type reflectance_request_avoidance_data;

// local variables
static const char *TAG = "reflectance";

/**
 * @brief Sensors, in order: front left, front right, back left, back right.
 */
static int64_t timer_measurements[NUMBER_OF_REFLECTANCE_SENSORS];
static bool sensor_active[NUMBER_OF_REFLECTANCE_SENSORS];
static int sensor_voltage[NUMBER_OF_REFLECTANCE_SENSORS];
static gpio_num_t reflectance_gpios[NUMBER_OF_REFLECTANCE_SENSORS] = {
    GPIO_NUM_36,
    GPIO_NUM_39,
    GPIO_NUM_34,
    GPIO_NUM_35};

static adc_channel_t reflectance_adc_channels[NUMBER_OF_REFLECTANCE_SENSORS] = {
    ADC_CHANNEL_0,
    ADC_CHANNEL_3,
    ADC_CHANNEL_6,
    ADC_CHANNEL_7};
static uint8_t timer_measurement_indexes[NUMBER_OF_REFLECTANCE_SENSORS] = {0, 1, 2, 3};
static int64_t measurement_start_time;

static bool front_right_warning = false;
static bool front_left_warning = false;

// inline function declarations

// function definitions
static void reflectance_measure();
static void reflectance_arbitrate_state();
static void reflectance_output();

// function declarations

void reflectance_init()
{
    for (int i = 0; i < NUMBER_OF_REFLECTANCE_SENSORS; ++i)
    {
        // gpio_set_intr_type(reflectance_gpios[i], GPIO_INTR_NEGEDGE);
        gpio_set_direction(reflectance_gpios[i], GPIO_MODE_INPUT);
    }
}

TASK reflectance_main()
{
    for (;;)
    {
        reflectance_measure();
        // ESP_LOGI(TAG, "%lli %lli %lli %lli",
        //          timer_measurements[0],
        //          timer_measurements[1],
        //          timer_measurements[2],
        //          timer_measurements[3]);

        // ESP_LOGI(TAG, "%d %d %d %d",
        //          sensor_voltage[0],
        //          sensor_voltage[1],
        //          sensor_voltage[2],
        //          sensor_voltage[3]);

        reflectance_output();

        vTaskDelay(1000);
    }
}

void reflectance_measure()
{
    /**
     * @brief Workaround so that we do not get incorrect interrupts on GPIO36 and GPIO39.
     */
    adc_power_acquire();

    // for (int i = 0; i < NUMBER_OF_REFLECTANCE_SENSORS; ++i)
    // {
    //     timer_measurements[i] = 0;

    //     gpio_isr_handler_add(reflectance_gpios[i], reflectance_isr, &timer_measurement_indexes[i]);
    //     gpio_set_pull_mode(reflectance_gpios[i], GPIO_PULLUP_ONLY);
    //     // gpio_set_direction(reflectance_gpios[i], GPIO_MODE_OUTPUT);
    //     // gpio_set_level(reflectance_gpios[i], 1);
    // }

    // vTaskDelay(pdMS_TO_TICKS(REFLECTANCE_SENSOR_MEASUREMENT_PREWAIT));

    // measurement_start_time = esp_timer_get_time();
    // for (int i = 0; i < NUMBER_OF_REFLECTANCE_SENSORS; ++i)
    // {
    //     gpio_set_pull_mode(reflectance_gpios[i], GPIO_PULLDOWN_ONLY);
    //     // gpio_set_direction(reflectance_gpios[i], GPIO_MODE_INPUT);
    // }

    // vTaskDelay(pdMS_TO_TICKS(REFLECTANCE_SENSOR_MEASUREMENT_WAIT));

    // for (int i = 0; i < NUMBER_OF_REFLECTANCE_SENSORS; ++i)
    // {
    //     timer_measurements[i] = esp_timer_get_time() - timer_measurements[i];
    //     /**
    //      * @brief Disable GPIO interrupts to make sure they will not trigger
    //      * when waiting for the next measurement.
    //      */
    //     gpio_isr_handler_remove(reflectance_gpios[i]);
    // }
    // // ESP_LOGI(TAG, "%lli us", timer_measurements[0]);
    // for (int i = 0; i < NUMBER_OF_REFLECTANCE_SENSORS; ++i)
    // {
    //     if (timer_measurements[i] > REFLECTANCE_THRESHOLD)
    //     {
    //         sensor_active[i] = true;
    //     }
    //     else
    //     {
    //         sensor_active[i] = false;
    //     }
    // }

    // for (int i = 0; i < NUMBER_OF_REFLECTANCE_SENSORS; ++i)
    // {
    //     sensor_active[i] = gpio_get_level(reflectance_gpios[i]);
    // }

    for (int i = 0; i < NUMBER_OF_REFLECTANCE_SENSORS; ++i)
    {
        sensor_voltage[i] = adc1_get_raw(reflectance_adc_channels[i]);
    }

    /**
     * @brief Workaround so that we do not get incorrect interrupts on GPIO36 and GPIO39.
     */
    adc_power_release();
}

void reflectance_output()
{
    if (sensor_active[0])
    {
        front_left_warning = true;
    }
    if (sensor_active[1])
    {
        front_right_warning = true;
    }

    if (sensor_active[2] && front_left_warning)
    {
        /**
         * @brief Detected line crossing on the left of the vehicle.
         * This flag will be cleared in algo after receiving it.
         */
        reflectance_request_avoidance_data.left = REQUEST_AVOIDANCE;

        /**
         * @brief Clear front_left_warning flag.
         */
        front_left_warning = false;
    }

    if (sensor_active[3] && front_right_warning)
    {
        /**
         * @brief Detected line crossing on the right of the vehicle.
         * This flag will be cleared in algo after receiving it.
         */
        reflectance_request_avoidance_data.right = REQUEST_AVOIDANCE;

        /**
         * @brief Clear front_right_warning flag.
         */
        front_right_warning = false;
    }
}