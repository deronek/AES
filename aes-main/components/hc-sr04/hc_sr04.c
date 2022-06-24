#include "hc_sr04.h"

#include "ble.h"

#include "driver/rmt.h"

// constants
/**
 * @brief Time to wait after every measurement (to decrease probability of inter-sensor disruptions)
 */
#define MEASUREMENT_DELAY_TIME pdMS_TO_TICKS(10)
#define HC_SR04_TIMEOUT_MS (40)
#define HC_SR04_TIMEOUT_TICKS pdMS_TO_TICKS(HC_SR04_TIMEOUT_MS)

#define NUMBER_OF_TRIG_PINS (NUMBER_OF_HC_SR04_SENSORS / 2)
#define NUMBER_OF_ECHO_PINS 2

#define RMT_TX_CHANNEL 1 /* RMT channel for transmitter */
// #define RMT_TX_GPIO_NUM PIN_TRIGGER /* GPIO number for transmitter signal */
#define RMT_RX_CHANNEL 0 /* RMT channel for receiver */
// #define RMT_RX_GPIO_NUM PIN_ECHO    /* GPIO number for receiver */
#define RMT_CLK_DIV 100     /* RMT counter clock divider */
#define RMT_TX_CARRIER_EN 0 /* Disable carrier */
/**
 * @todo fix this time
 *
 */
#define rmt_item32_tIMEOUT_US (HC_SR04_TIMEOUT_MS * 1000)                                                                  /*!< RMT receiver timeout value(us) */
#define RMT_TICK_10_US (80000000UL / RMT_CLK_DIV / 100000UL) /* RMT counter value for 10 us.(Source clock is APB clock) */ // 80
#define ITEM_DURATION(d) ((d & 0x7fff) * 10 / RMT_TICK_10_US)

#define DISTANCE_MAX (2000000)
#define DISTANCE_MIN (2000)

// structs
typedef struct hc_sr04_pair_type_tag
{
    uint8_t id0, id1;
} hc_sr04_pair_type;

typedef struct hc_sr04_gpio_type_tag
{
    gpio_num_t trig_pin;
    gpio_num_t echo_pin;
} hc_sr04_gpio_type;

// global variables

// local variables

static const char *TAG = "hc_sr04";

QueueHandle_t hc_sr04_queue_data;

// local sensor data
hc_sr04_data_type hc_sr04_data;

// hc_sr04_gpio_type hc_sr04_gpio[NUMBER_OF_HC_SR04_SENSORS] = {
//     {GPIO_NUM_25,
//      GPIO_NUM_26},
//     {GPIO_NUM_32,
//      GPIO_NUM_33}};

uint8_t hc_sr04_trig_pins[NUMBER_OF_TRIG_PINS] = {
    GPIO_NUM_33,
    GPIO_NUM_25,
    GPIO_NUM_26,
    GPIO_NUM_27};

uint8_t hc_sr04_echo_pins[NUMBER_OF_ECHO_PINS] = {
    GPIO_NUM_14,
    GPIO_NUM_12};

/**
 * @brief Hardcoded distance offset for each sensor.
 * This should be equal to distance of the sensor from the vehicle "bubble".
 * Units are micrometers.
 */
uint32_t hc_sr04_distance_offset[NUMBER_OF_HC_SR04_SENSORS] = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0};

/**
 * @brief Mapping of sensor indexes (starting from the left).
 */
#define HC_SR04_MAP_SENSORS
uint8_t hc_sr04_sensor_mapping[NUMBER_OF_HC_SR04_SENSORS] = {
    0, // 1st
    4, // 5th
    1, // 2nd
    5, // 6th
    2, // 3rd
    6, // 7th
    3, // 4th
    7, // 8th
};

RingbufHandle_t hc_sr04_rb_handles[NUMBER_OF_ECHO_PINS];

// hc_sr04_gpio_type hc_sr04_gpio[NUMBER_OF_HC_SR04_SENSORS] = {{GPIO_NUM_25, GPIO_NUM_26}, {GPIO_NUM_25, GPIO_NUM_33}};

/**
 * @brief Pairs of HC-SR04 sensors which should be polled simultaneously.
 * Indexed from 0 to NUMBER_OF_HC_SR04_SENSORS - 1.
 * @todo Move all of this setup from hardcoded to ESP-IDF configuration.
 */
// hc_sr04_pair_type hc_sr04_pairs[NUMBER_OF_HC_SR04_PAIRS] = {
//     {0, 1}};

static uint64_t sensor0_start, sensor0_end, sensor1_start, sensor1_end;
static bool sensor0_done, sensor1_done;

// inline function declarations
inline static uint32_t hc_sr04_calculate_distance(uint32_t time)
{
    if (time == 0)
    {
        return 0;
    }

    // rescale time to microseconds
    time *= 1000;
    time /= 800;
    // time /= 8;

    // distance in micrometers
    /**
     * @todo Implement speed of sound variable based on the temperature.
     * Remember to divide it by 2.
     */
    uint32_t distance = time * 172;
    return distance;
}

// function declarations

void hc_sr04_init()
{
    /**
     * @brief Initialize all trig pins
     */
    for (int i = 0; i < NUMBER_OF_TRIG_PINS; ++i)
    {
        uint8_t pin = hc_sr04_trig_pins[i];

        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(pin, GPIO_FLOATING);

        ESP_LOGI(TAG, "Initialized trig pin at GPIO %d", pin);
    }

    /**
     * @brief Initialize all echo pins
     */

    for (int i = 0; i < NUMBER_OF_ECHO_PINS; ++i)
    {
        uint8_t pin = hc_sr04_echo_pins[i];

        gpio_reset_pin(pin);
        gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY);

        rmt_config_t rmt_rx;
        rmt_rx.channel = i;
        rmt_rx.gpio_num = pin;
        rmt_rx.clk_div = RMT_CLK_DIV;
        rmt_rx.mem_block_num = 1;
        rmt_rx.rmt_mode = RMT_MODE_RX;
        rmt_rx.rx_config.filter_en = true;
        rmt_rx.rx_config.filter_ticks_thresh = 100;
        // rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
        rmt_rx.rx_config.idle_threshold = 9280; // 2 m distance
        rmt_config(&rmt_rx);
        rmt_driver_install(rmt_rx.channel, 1000, 0);

        rmt_get_ringbuf_handle(i, &(hc_sr04_rb_handles[i]));

        ESP_LOGI(TAG, "Initialized echo pin at GPIO %d, RB handle: %p", pin, hc_sr04_rb_handles[i]);
    }

    // for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
    // {
    //     // GPIO init
    //     hc_sr04_gpio_type sensor_gpio = hc_sr04_gpio[i];

    //     // TRIG pin
    //     gpio_reset_pin(sensor_gpio.trig_pin);
    //     gpio_set_direction(sensor_gpio.trig_pin, GPIO_MODE_OUTPUT);

    //     /**
    //      * @brief We do not need ECHO pin initialization
    //      * when we use RMT.
    //      */
    //     /*
    //     // ECHO pin
    //     gpio_reset_pin(sensor_gpio.echo_pin);
    //     gpio_set_direction(sensor_gpio.echo_pin, GPIO_MODE_INPUT);
    //     gpio_set_intr_type(sensor_gpio.echo_pin, GPIO_INTR_ANYEDGE);

    //     // out init
    //     hc_sr04_data.time[i] = HC_SR04_INIT_VALUE;*/

    //     rmt_config_t rmt_rx;
    //     rmt_rx.channel = i;
    //     rmt_rx.gpio_num = sensor_gpio.echo_pin;
    //     rmt_rx.clk_div = RMT_CLK_DIV;
    //     rmt_rx.mem_block_num = 1;
    //     rmt_rx.rmt_mode = RMT_MODE_RX;
    //     rmt_rx.rx_config.filter_en = true;
    //     rmt_rx.rx_config.filter_ticks_thresh = 100;
    //     rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
    //     rmt_config(&rmt_rx);
    //     rmt_driver_install(rmt_rx.channel, 1000, 0);
    // }

    /**
     * @brief Construct data queue
     */
    // hc_sr04_queue_data = malloc(sizeof(QueueHandle_t));
    // if (hc_sr04_queue_data == NULL)
    // {
    //     abort();
    // }
    if ((hc_sr04_queue_data = xQueueCreate(1, sizeof(hc_sr04_data_type))) == NULL)
    {
        abort();
    }
}

TASK hc_sr04_measure()
{
    size_t rx_size = 0;
    // ESP_LOGI(TAG, "RB data %p %p", (void *)data, (void *)(&data));

    // ESP_LOGI(TAG, "Handles %p", &hc_sr04_rb_handles);
    // ESP_LOGI(TAG, "Queue %p", &hc_sr04_queue_data);
    // ESP_LOGI(TAG, "Data %p", &hc_sr04_data);

    for (;;)
    {
        int val;

        /**
         * @todo Refactor this to use sensor pairs.
         * Think of how to actually connect 8 sensors.
         * Because of the pairs, we can probably only use
         * 2 pins for ECHO (add diodes of course).
         *
         * So, can we also only use 4 pins for TRIG? (triggering
         * pair at the same time).
         * Sensor will be connected probably be connected
         * in pairs like 1-5, 2-6, 3-7, 4-8 to minimize
         * sensor intereference.
         */
        for (int trig = 0; trig < NUMBER_OF_TRIG_PINS; ++trig)
        {
            // ESP_LOGI(TAG, "Trig %d", trig);
            /**
             * @brief We won't use TX channel of RMT, because it only
             * has 8 channels and we will probably need eight different
             * channels to send.
             */
            uint8_t trig_pin = hc_sr04_trig_pins[trig];
            // rmt_write_items(RMT_TX_CHANNEL, &item, 1, true);
            // rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);
            /**
             * @todo Do we really want to start the RX here? Maybe before sending
             * trigger?
             */

            gpio_set_level(trig_pin, 1);
            /**
             * @todo Change this delay to actually 10 us if possible.
             * Current implementation is slower this way and it is probable
             * that we miss some echo pulses this way.
             * We can do it via interrupt and High Resolution timer.
             */
            vTaskDelay(pdMS_TO_TICKS(1));
            gpio_set_level(trig_pin, 0);

            for (int echo = 0; echo < NUMBER_OF_ECHO_PINS; ++echo)
            {
                rmt_rx_start(echo, 1);
            }

            /**
             * @todo We wait for the first item to receive, then we attempt to receive the second.
             * There probably is not way to optimize this, since we do not want to move forward until
             * both sensor data are received (or timeout happens).
             */
            uint32_t rx_durations[NUMBER_OF_ECHO_PINS];
            TickType_t ticks_to_wait = HC_SR04_TIMEOUT_TICKS;
            TickType_t start = xTaskGetTickCount();
            for (int echo = 0; echo < NUMBER_OF_ECHO_PINS; ++echo)
            {
                // size_t a = xRingbufferGetCurFreeSize(hc_sr04_rb_handles[echo]);
                // ESP_LOGI(TAG, "Current free size: %d", a);
                // xRingbufferPrintInfo(hc_sr04_rb_handles[echo]);
                // data[echo] = (rmt_item32_t *)xRingbufferReceive(hc_sr04_rb_handles[echo], &rx_size, ticks_to_wait);
                // ESP_LOGI(TAG, "Receiving on RB %p", hc_sr04_rb_handles[echo]);
                rmt_item32_t *ptr = (rmt_item32_t *)xRingbufferReceive(hc_sr04_rb_handles[echo], &rx_size, ticks_to_wait);
                rmt_rx_stop(echo);
                if (ptr)
                {
                    rx_durations[echo] = ptr->duration0;
                    vRingbufferReturnItem(hc_sr04_rb_handles[echo], (void *)ptr);
                }
                /**
                 * @brief If you compile code with below else block (even if it only has NOP) with -Og optimization setting,
                 * sensor measurement will not work. Use -Os or -O2 optimization setting.
                 *
                 * Could not pinpoint the reason of this issue, no debugger available.
                 * Assembly analysis doesn't show any issue, looks correct.
                 */
                else
                {
                    rx_durations[echo] = 0;
                    // _asm__ __volatile__("nop");
                }
                // ESP_LOGI(TAG, "Size: %d", rx_size);

                /**
                 * @brief Decrease time to wait for next sensors, because we already waited that time.
                 * This is unsigned arithmetic, so it should give correct value even if FreeRTOS
                 * tick count overflows.
                 */
                ticks_to_wait -= xTaskGetTickCount() - start;
            }

            // /**
            //  * @todo Maybe refactor distance calculation for other
            //  * situation than pairs. Just hardcoded for 2 sensors here.
            //  */
            // if (data[0])
            //     hc_sr04_data.distance[trig] = hc_sr04_calculate_distance(data[0]->duration0);
            // vRingbufferReturnItem(rb[i], (void *)item);
            // else hc_sr04_data.distance[trig] = 0;

            // if (data[1])
            //     hc_sr04_data.distance[trig + NUMBER_OF_TRIG_PINS] = hc_sr04_calculate_distance(data[1]->duration0);
            // else
            //     hc_sr04_data.distance[trig + NUMBER_OF_TRIG_PINS] = 0;

            for (int echo = 0; echo < NUMBER_OF_ECHO_PINS; ++echo)
            {
                size_t measurement_index = echo + trig * NUMBER_OF_ECHO_PINS;

#ifdef HC_SR04_MAP_SENSORS
                measurement_index = hc_sr04_sensor_mapping[measurement_index];
#endif
                /**
                 * @brief Calculate distance measurement from pulse duration.
                 * If measurement was not successful, put 2 meters (no obstacle).
                 */
                volatile uint32_t distance;
                if (rx_durations[echo])
                {
                    distance = hc_sr04_calculate_distance(rx_durations[echo]) - hc_sr04_distance_offset[measurement_index];
                }
                else
                {
                    distance = DISTANCE_MAX;
                }

                if (distance < DISTANCE_MIN)
                {
                    distance = DISTANCE_MAX;
                }

                // ESP_LOGI(TAG, "Distance %d", distance);

                // ESP_LOGI(TAG, "Distance %d", distance);
                hc_sr04_data.distance[measurement_index] = distance;
            }

            // ESP_LOGI(TAG, "Sensor %d - distance %d", i, hc_sr04_data.distance[i]);

            vTaskDelay(MEASUREMENT_DELAY_TIME);
        }
        // ESP_LOGI(TAG, "Sending data to queue");
        // ESP_LOGI(TAG, "%u %u", hc_sr04_data.distance[0], hc_sr04_data.distance[1]);
        ESP_LOGI(TAG, "%u %u %u %u %u %u %u %u", hc_sr04_data.distance[0], hc_sr04_data.distance[1], hc_sr04_data.distance[2], hc_sr04_data.distance[3],
                 hc_sr04_data.distance[4], hc_sr04_data.distance[5], hc_sr04_data.distance[6], hc_sr04_data.distance[7]);

        // ESP_LOGI(TAG, "%u %u %u %u", hc_sr04_data.distance[0], hc_sr04_data.distance[1], hc_sr04_data.distance[2], hc_sr04_data.distance[3]);

        // ESP_LOGI(TAG, "Queue address %p", hc_sr04_queue_data);
        ble_send_from_task(TASK_ID_HC_SR04, &hc_sr04_data);
        xQueueOverwrite(hc_sr04_queue_data, &hc_sr04_data);

        // app_manager_algo_task_notify(TASK_FLAG_HC_SR04);
        // app_manager_ble_task_notify(TASK_FLAG_HC_SR04);
    }
}