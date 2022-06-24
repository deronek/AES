#include "reflectance.h"
#include "driver/adc.h"
#include "driver/gpio.h"

// constants

#define NUMBER_OF_REFLECTANCE_SENSORS (2)

#define REFLECTANCE_SENSOR_MEASUREMENT_PREWAIT (pdMS_TO_TICKS(5))
#define REFLECTANCE_SENSOR_MEASUREMENT_WAIT (pdMS_TO_TICKS(50))

#define TASK_TICK_PERIOD (pdMS_TO_TICKS(100))

/**
 * @todo Fill in the value.
 */
#define REFLECTANCE_THRESHOLD (500)

// structs

// global variables
reflectance_request_avoidance_type reflectance_request_avoidance;

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

        vTaskDelay(TASK_TICK_PERIOD);
    }
}

void reflectance_measure()
{
    /**
     * @brief Workaround so that we do not get incorrect interrupts on GPIO36 and GPIO39.
     */
    adc_power_acquire();

    for (int i = 0; i < NUMBER_OF_REFLECTANCE_SENSORS; ++i)
    {
        sensor_voltage[i] = adc1_get_raw(reflectance_adc_channels[i]);

        if (sensor_voltage[i] < REFLECTANCE_THRESHOLD)
        {
            sensor_active[i] = true;
        }
        else
        {
            sensor_active[i] = false;
        }
    }

    /**
     * @brief Workaround so that we do not get incorrect interrupts on GPIO36 and GPIO39.
     */
    adc_power_release();
}

/**
 * @brief Avoidance request is set here and will be cleared in algo.
 */
void reflectance_output()
{
    if (sensor_active[0])
    {
        reflectance_request_avoidance.left = true;
    }

    if (sensor_active[1])
    {
        reflectance_request_avoidance.right = true;
    }
}