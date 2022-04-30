#include "hc_sr04.h"

// constants
/**
 * @brief Time to wait after every measurement (to decrease probability of inter-sensor disruptions)
 */
#define MEASUREMENT_DELAY_TIME pdMS_TO_TICKS(10)

#define NUMBER_OF_HC_SR04_PAIRS (NUMBER_OF_HC_SR04_SENSORS / 2)

#define RMT_TX_CHANNEL 1                                 /* RMT channel for transmitter */
#define RMT_TX_GPIO_NUM PIN_TRIGGER                      /* GPIO number for transmitter signal */
#define RMT_RX_CHANNEL 0                                 /* RMT channel for receiver */
#define RMT_RX_GPIO_NUM PIN_ECHO                         /* GPIO number for receiver */
#define RMT_CLK_DIV 100                                  /* RMT counter clock divider */
#define RMT_TX_CARRIER_EN 0                              /* Disable carrier */
#define rmt_item32_tIMEOUT_US 500000                     /*!< RMT receiver timeout value(us) */
#define RMT_TICK_10_US (80000000 / RMT_CLK_DIV / 100000) /* RMT counter value for 10 us.(Source clock is APB clock) */
#define ITEM_DURATION(d) ((d & 0x7fff) * 10 / RMT_TICK_10_US)

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

hc_sr04_gpio_type hc_sr04_gpio[NUMBER_OF_HC_SR04_SENSORS] = {
    {GPIO_NUM_25,
     GPIO_NUM_26},
    {GPIO_NUM_32,
     GPIO_NUM_33}};

/**
 * @brief Pairs of HC-SR04 sensors which should be polled simultaneously.
 * Indexed from 0 to NUMBER_OF_HC_SR04_SENSORS - 1.
 * @todo Move all of this setup from hardcoded to ESP-IDF configuration.
 */
hc_sr04_pair_type hc_sr04_pairs[NUMBER_OF_HC_SR04_PAIRS] = {
    {0, 1}};

static uint64_t sensor0_start, sensor0_end, sensor1_start, sensor1_end;
static bool sensor0_done, sensor1_done;

// function declarations

void hc_sr04_init()
{
    for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
    {
        // GPIO init
        hc_sr04_gpio_type sensor_gpio = hc_sr04_gpio[i];

        // TRIG pin
        gpio_reset_pin(sensor_gpio.trig_pin);
        gpio_set_direction(sensor_gpio.trig_pin, GPIO_MODE_OUTPUT);

        /**
         * @brief We do not need ECHO pin initialization
         * when we use RMT.
         */
        /*
        // ECHO pin
        gpio_reset_pin(sensor_gpio.echo_pin);
        gpio_set_direction(sensor_gpio.echo_pin, GPIO_MODE_INPUT);
        gpio_set_intr_type(sensor_gpio.echo_pin, GPIO_INTR_ANYEDGE);

        // out init
        hc_sr04_data.time[i] = HC_SR04_INIT_VALUE;*/

        rmt_config_t rmt_rx;
        rmt_rx.channel = i;
        rmt_rx.gpio_num = sensor_gpio.echo_pin;
        rmt_rx.clk_div = RMT_CLK_DIV;
        rmt_rx.mem_block_num = 1;
        rmt_rx.rmt_mode = RMT_MODE_RX;
        rmt_rx.rx_config.filter_en = true;
        rmt_rx.rx_config.filter_ticks_thresh = 100;
        rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
        rmt_config(&rmt_rx);
        rmt_driver_install(rmt_rx.channel, 1000, 0);

        ESP_LOGI(TAG, "Initialized ultrasonic sensor at pins: TRIG - %d, ECHO - %d", sensor_gpio.trig_pin, sensor_gpio.echo_pin);
    }

    // create queue
    if ((hc_sr04_queue_data = xQueueCreate(1, sizeof(hc_sr04_data_type))) == NULL)
    {
        abort();
    }
}

TASK hc_sr04_measure()
{
    size_t rx_size = 0;
    RingbufHandle_t rb[NUMBER_OF_HC_SR04_SENSORS];
    /**
     * @todo Move these ring buffers to init, also refactor
     * channel numbers for multiple sensor usage
     */
    rmt_get_ringbuf_handle(0, &(rb[0]));
    rmt_get_ringbuf_handle(1, &(rb[1]));

    for (;;)
    {
        int val;
        uint64_t time;

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
        for (int i = 0; i < NUMBER_OF_HC_SR04_SENSORS; ++i)
        {
            /**
             * @brief We won't use TX channel of RMT, because it only
             * has 8 channels and we will probably need eight different
             * channels to send.
             */
            // rmt_write_items(RMT_TX_CHANNEL, &item, 1, true);
            // rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);
            hc_sr04_gpio_type sensor_gpio = hc_sr04_gpio[i];
            gpio_set_level(sensor_gpio.trig_pin, 1);
            /**
             * @todo Change this delay to actually 10 us if possible.
             * Current implementation is slower this way and it is probable
             * that we miss some echo pulses this way.
             */
            vTaskDelay(pdMS_TO_TICKS(1));
            gpio_set_level(sensor_gpio.trig_pin, 0);
            /**
             * @todo Do we really want to start the RX here? Maybe before sending
             * trigger?
             */
            rmt_rx_start(i, 1);

            /**
             * @todo Change tick time used to wait here, should probably be max timeout of sensor
             */
            rmt_item32_t *item = (rmt_item32_t *)xRingbufferReceive(rb[i], &rx_size, 1000);
            rmt_rx_stop(i);

            hc_sr04_data.time[i] = item->duration0;

            // ESP_LOGI(TAG, "Sensor %d: level0 %d, duration0 %d, level1 %d, duration1 %d",
            //          i, item->level0, item->duration0, item->level1, item->duration1);

            vRingbufferReturnItem(rb[i], (void *)item);
            vTaskDelay(MEASUREMENT_DELAY_TIME);
        }
        // ESP_LOGI(TAG, "Sending data to queue");
        xQueueOverwrite(hc_sr04_queue_data, &hc_sr04_data);
        algo_task_notify(TASK_FLAG_HC_SR04);
        ble_task_notify(TASK_FLAG_HC_SR04);
    }
}