#include "reflectance.h"
#include "driver/adc.h"
#include "driver/gpio.h"

#include "algo.h"

#include <math.h>

// constants

#define NUMBER_OF_REFLECTANCE_SENSORS (2)

#define REFLECTANCE_SENSOR_MEASUREMENT_PREWAIT (pdMS_TO_TICKS(5))
#define REFLECTANCE_SENSOR_MEASUREMENT_WAIT (pdMS_TO_TICKS(50))

#define REFLECTANCE_TRIG_PIN (GPIO_NUM_32)

#define TASK_TICK_PERIOD (pdMS_TO_TICKS(50))

/**
 * @todo Fill in the value.
 */
// #define REFLECTANCE_THRESHOLD_L (500)
// #define REFLECTANCE_THRESHOLD_L (1500)
// #define REFLECTANCE_THRESHOLD_R (1000)
// #define REFLECTANCE_THRESHOLD_R (2500)

#define REFLECTANCE_THRESHOLD_DIFFERENCE_L (500)
#define REFLECTANCE_THRESHOLD_DIFFERENCE_R (500)

#define CALIBRATION_DATA_POINTS_EXP 5
#define CALIBRATION_DATA_POINTS (1 << CALIBRATION_DATA_POINTS_EXP)

// structs

// global variables
reflectance_request_avoidance_type reflectance_request_avoidance;
bool reflectance_measurement_enabled;
uint32_t ble_tick_send_counter;

// local variables
static const char *TAG = "reflectance";
static int sensor_bias[NUMBER_OF_REFLECTANCE_SENSORS];
bool reflectance_stop_requested;

/**
 * @brief Sensors, in order: front left, front right, back left, back right.
 */
// static bool sensor_active[NUMBER_OF_REFLECTANCE_SENSORS];
static int last_sensor_voltage[NUMBER_OF_REFLECTANCE_SENSORS];
static int sensor_voltage[NUMBER_OF_REFLECTANCE_SENSORS];
static gpio_num_t reflectance_gpios[NUMBER_OF_REFLECTANCE_SENSORS] = {
    GPIO_NUM_35,
    GPIO_NUM_36};

static adc_channel_t reflectance_adc_channels[NUMBER_OF_REFLECTANCE_SENSORS] = {
    ADC_CHANNEL_7,
    ADC_CHANNEL_0};

// gpio 36 prawy, gpio 35 lewy
// inline function declarations

// function definitions
static void reflectance_measure();
static void reflectance_arbitrate_state();
static void reflectance_output();
static void reflectance_ble_send();

// function declarations

void reflectance_init()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    for (int i = 0; i < NUMBER_OF_REFLECTANCE_SENSORS; ++i)
    {
        // gpio_set_intr_type(reflectance_gpios[i], GPIO_INTR_NEGEDGE);
        // gpio_set_direction(reflectance_gpios[i], GPIO_MODE_INPUT);
        // gpio_set_pull_mode(reflectance_gpios[i], GPIO_FLOATING);
        adc1_config_channel_atten(reflectance_adc_channels[i], ADC_ATTEN_11db);
    }

    gpio_set_direction(REFLECTANCE_TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(REFLECTANCE_TRIG_PIN, 1);
}

TASK reflectance_main()
{
    for (;;)
    {
        if (reflectance_stop_requested)
        {
            break;
        }
        reflectance_measure();
        ESP_LOGI(TAG, "%d %d",
                 sensor_voltage[0],
                 sensor_voltage[1]);

        if (reflectance_measurement_enabled)
        {
            reflectance_output();
        }

        // if (ble_tick_send_counter >= 10)
        // {
        //     reflectance_ble_send();
        //     ble_tick_send_counter = 0;
        // }
        // else
        // {
        //     ble_tick_send_counter++;
        // }

        vTaskDelay(TASK_TICK_PERIOD);
    }

    ESP_LOGI(TAG, "Suspending task");
    xTaskNotifyGive(app_manager_algo_task_handle);
    vTaskSuspend(NULL);
}

void reflectance_ble_send()
{
    ble_send_from_task(TASK_ID_REFLECTANCE, &reflectance_request_avoidance);
}

void reflectance_calibrate()
{
    for (int i = 0; i < NUMBER_OF_REFLECTANCE_SENSORS; ++i)
    {
        sensor_bias[i] = 0;
    }

    for (int i = 0; i < CALIBRATION_DATA_POINTS; ++i)
    {
        for (int p = 0; p < NUMBER_OF_REFLECTANCE_SENSORS; ++p)
        {
            sensor_bias[p] += adc1_get_raw(reflectance_adc_channels[p]);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    for (int i = 0; i < NUMBER_OF_REFLECTANCE_SENSORS; ++i)
    {
        sensor_bias[i] = (int)roundf((float)sensor_bias[i] / CALIBRATION_DATA_POINTS);
        ESP_LOGI(TAG, "Calibration sensor %d bias: %d", i, sensor_bias[i]);
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
        last_sensor_voltage[i] = sensor_voltage[i];
        sensor_voltage[i] = adc1_get_raw(reflectance_adc_channels[i]);
        sensor_voltage[i] -= sensor_bias[i];

        // if (sensor_voltage[i] < REFLECTANCE_THRESHOLD)
        // {
        //     sensor_active[i] = true;
        // }
        // else
        // {
        //     sensor_active[i] = false;
        // }
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
    // if (sensor_voltage[0] < REFLECTANCE_THRESHOLD_L)
    // {
    //     reflectance_request_avoidance.left = true;
    // }

    // if (sensor_voltage[1] < REFLECTANCE_THRESHOLD_R)
    // {
    //     reflectance_request_avoidance.right = true;
    // }

    if ((last_sensor_voltage[0] - sensor_voltage[0]) > REFLECTANCE_THRESHOLD_DIFFERENCE_L)
    {
        ESP_LOGW(TAG, "Left sensor triggered");
        reflectance_request_avoidance.left = true;
    }

    if ((last_sensor_voltage[1] - sensor_voltage[1]) > REFLECTANCE_THRESHOLD_DIFFERENCE_R)
    {
        ESP_LOGW(TAG, "Right sensor triggered");
        reflectance_request_avoidance.right = true;
    }
}

void reflectance_reset()
{
    reflectance_request_avoidance.left = false;
    reflectance_request_avoidance.right = false;
    reflectance_measurement_enabled = false;
    reflectance_stop_requested = false;
    ble_tick_send_counter = 0;
}

void reflectance_request_stop()
{
    reflectance_stop_requested = true;
}