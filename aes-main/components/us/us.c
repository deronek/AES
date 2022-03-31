#include "us.h"

// global variables

us_out_type us_out[NUMBER_OF_USS];

// local variables

static const char *TAG = "us";

us_gpio_type us_gpio[NUMBER_OF_USS] = {
    {
        GPIO_NUM_27,
        GPIO_NUM_26
    }
};

bool timeout;

// function declarations

static bool IRAM_ATTR set_sensor_timeout();
static void us_reset_measurement();

// function definitions

static bool IRAM_ATTR set_sensor_timeout()
{
    timeout = true;
    return true;
}

static void us_reset_measurement()
{
    timer_pause(TIMER_GROUP_NUM, TIMER_IDX_NUM);
    timer_set_counter_value(TIMER_GROUP_NUM, TIMER_IDX_NUM, 0);
    timeout = false;
}

void us_measure()
{
    int val;
    uint64_t time;

    for(int i = 0; i < NUMBER_OF_USS; ++i)
    {
        us_gpio_type sensor_gpio = us_gpio[i];

        // send TRIG
        gpio_set_level(sensor_gpio.trig_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(sensor_gpio.trig_pin, 0);
        timer_start(TIMER_GROUP_NUM, TIMER_IDX_NUM);

        do {
            val = gpio_get_level(sensor_gpio.echo_pin);
        } while (val == 0 && !timeout);
        if (timeout)
        {
            us_out[i].status = US_OUT_TIMEOUT_ECHO_START;
            us_reset_measurement();
            continue;
        }

        timeout = false;
        timer_set_counter_value(TIMER_GROUP_NUM, TIMER_IDX_NUM, 0);

        do {
            val = gpio_get_level(sensor_gpio.echo_pin);
            timer_get_counter_value(TIMER_GROUP_NUM, TIMER_IDX_NUM, &time);
        } while (val == 1 && !timeout);

        if (timeout)
        {
            us_out[i].status = US_OUT_TIMEOUT_ECHO_END;
            us_reset_measurement();
            continue;
        }

        // measured correctly

        us_reset_measurement();

        us_out[i].status = US_OUT_OK;
        us_out[i].value = time;
    }
}

void us_init()
{
    for(int i = 0; i < NUMBER_OF_USS; ++i)
    {
        // GPIO init
        us_gpio_type sensor_gpio = us_gpio[i];

        // TRIG pin
        gpio_reset_pin(sensor_gpio.trig_pin);
        gpio_set_direction(sensor_gpio.trig_pin, GPIO_MODE_OUTPUT);

        // ECHO pin
        gpio_reset_pin(sensor_gpio.echo_pin);
        gpio_set_direction(sensor_gpio.echo_pin, GPIO_MODE_INPUT);

        // out array init
        us_out[i].status = US_OUT_INIT;

        ESP_LOGI(TAG, "Initialized ultrasonic sensor at pins: TRIG - %d, ECHO - %d", sensor_gpio.trig_pin, sensor_gpio.echo_pin);
    }

    // General Purpose Timer init
    timer_config_t timer_config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        //.intr_type = TIMER_INTR_MAX,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = TIMER_DIVIDER
    };
    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_NUM, TIMER_IDX_NUM, &timer_config));
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_NUM, TIMER_IDX_NUM, TIMER_S_TO_TICKS(US_TIMEOUT_S)));
    ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_NUM, TIMER_IDX_NUM));
    ESP_ERROR_CHECK(timer_isr_callback_add(TIMER_GROUP_NUM, TIMER_IDX_NUM, set_sensor_timeout, NULL, 0));
    timeout = false;

    ESP_LOGI(TAG, "Initialized General Purpose Timer, group %d, index %d", TIMER_GROUP_NUM, TIMER_IDX_NUM);
}