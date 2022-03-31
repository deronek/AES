#pragma once

#ifndef _US_H
#define _US_H

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/timer.h"

#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc.h"

// constants

#define NUMBER_OF_USS 1
#define US_TIMEOUT_S 0.03

#define TIMER_GROUP_NUM TIMER_GROUP_0
#define TIMER_IDX_NUM TIMER_0
#define TIMER_DIVIDER 2

#define TIMER_TICKS_TO_S(ticks) (ticks * TIMER_DIVIDER / rtc_clk_apb_freq_get())
#define TIMER_S_TO_TICKS(s) (s * rtc_clk_apb_freq_get() / TIMER_DIVIDER)

// enums
typedef enum us_out_status_type_tag
{
    US_OUT_OK,
    US_OUT_TIMEOUT_ECHO_START,
    US_OUT_TIMEOUT_ECHO_END,
    US_OUT_INIT 
} us_out_status_type;

// structs

typedef struct us_gpio_type_tag {
    gpio_num_t trig_pin;
    gpio_num_t echo_pin;
} us_gpio_type;

typedef struct us_out_type_tag {
    uint64_t value;
    us_out_status_type status;
} us_out_type;

// global variables

extern us_out_type us_out[NUMBER_OF_USS];

// function declarations

void us_measure();
void us_init();

#endif