#include "ble.h"

// global variables

// local variables

static const char *TAG = "ble";

// function defintions
static void ble_receive_data(uint8_t id, uint8_t *packet);
static void ble_handle_notification(uint32_t task_notification_value);
static uint32_t ble_get_task_id_from_flag(uint32_t task_id);
static void ble_prepare_and_send_packet(uint32_t task_id);
static esp_err_t ble_send_data(uint8_t *data, uint8_t len);

// function declarations

void ble_init()
{
}

TASK ble_main()
{
    for (;;)
    {
        /**
         * @brief
         * - wait for task notification from any sensor task
         * - get data from sensor queue
         * - send that data using ble_send_data()
         */
        uint32_t task_notification_value = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ble_handle_notification(task_notification_value);
    }
}

void ble_handle_notification(uint32_t task_notification_value)
{
    /**
     * @brief Iterate through the task notification value,
     * one byte at the time,
     * to send all of the packets which need to be sent.
     * We start from the LSB (highest priority).
     * We stop iterating when all requested packets are sent.
     */
    uint32_t bitmask = 1;
    while (task_notification_value)
    {
        uint32_t task_flag = task_notification_value & bitmask;
        if (task_flag)
        {
            uint32_t task_id = ble_get_task_id_from_flag(task_flag);
            ble_prepare_and_send_packet(task_id);
        }
        // clear bit of task that we just sent
        task_notification_value &= ~bitmask;

        // shift mask to the next index
        bitmask <<= 1;
    }
}

/**
 * @brief Prepare packet to send via BLE and send it using
 * ble_send_data function.
 *
 * Multi-byte data is send using system endianness (Xtensa LX6 is little-endian).
 *
 * Packet structure:
 * +--------------+---------------------------------------------------+----------+
 * | Byte number  |                      Content                      |   Type   |
 * +--------------+---------------------------------------------------+----------+
 * | 0            | Task ID - app_manager_task_id_type enum value     | uint8_t  |
 * | 1 to 4       | Timestamp - number of FreeRTOS ticks from startup | uint32_t |
 * | 5            | Number of data bytes - n                          | uint8_t  |
 * | 6 to (n + 6) | Data bytes                                        | varying  |
 * | (n + 7)      | Checksum                                          | uint8_t  |
 * +--------------+---------------------------------------------------+----------+
 * @param task_flag
 */
void ble_prepare_and_send_packet(uint32_t task_id)
{
    uint8_t data_size = app_manager_task_data_size[task_id];
    uint8_t packet_length = 1 + 4 + 1 + data_size + 1;

    uint8_t *packet = malloc(packet_length);
    if (packet == NULL)
    {
        ESP_LOGE(TAG, "ble_prepare_and_send_packet malloc");
        abort();
    }

    packet[0] = task_id;

    uint32_t tick_count = xTaskGetTickCount();
    memcpy(packet + 1, &tick_count, sizeof(tick_count));

    packet[5] = data_size;

    ble_receive_data(task_id, packet + 6);

    /**
     * @todo Implement checksum - hardcoded value for now
     */
    packet[data_size + 1] = 255;

    ble_send_data(packet, packet_length);
}

#define LT(n) n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n
static const char LogTable256[256] =
    {
        -1, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3,
        LT(4), LT(5), LT(5), LT(6), LT(6), LT(6), LT(6),
        LT(7), LT(7), LT(7), LT(7), LT(7), LT(7), LT(7), LT(7)};

uint32_t ble_get_task_id_from_flag(uint32_t task_flag)
{
    /**
     * @brief Implemented using finding log base 2 - most future proof.
     * http://graphics.stanford.edu/~seander/bithacks.html
     */

    unsigned int v = task_flag;  // 32-bit word to find the log of
    unsigned r;                  // r will be lg(v)
    register unsigned int t, tt; // temporaries

    if ((tt = v >> 16))
    {
        r = (t = tt >> 8) ? 24 + LogTable256[t] : 16 + LogTable256[tt];
    }
    else
    {
        r = (t = v >> 8) ? 8 + LogTable256[t] : LogTable256[v];
    }

    return r;
}

void ble_receive_data(uint8_t id, uint8_t *destination)
{
    QueueHandle_t queue;
    switch (id)
    {
    case TASK_ID_HC_SR04:
        queue = hc_sr04_queue_data;
        break;
    case TASK_ID_MPU9255:
        queue = mpu9255_queue_fifo_data;
        break;
    default:
        abort();
    }
    xQueuePeek(queue, destination, 0);
}

esp_err_t ble_send_data(uint8_t *packet, uint8_t packet_length)
{
    // ESP_LOGI(TAG, "Sending packet, id %hhx, timestamp %hhx %hhx %hhx %hhx, data size %hhx", packet[0],
    //          packet[1], packet[2], packet[3], packet[4], packet[5]);
    // uint32_t dist1 = packet[9] | (packet[8] << 8) | (packet[7] << 16) | (packet[6] << 24);
    // uint32_t dist2 = packet[10] | (packet[11] << 8) | (packet[12] << 16) | (packet[13] << 24);

    // ESP_LOGI(TAG, "%u %u", dist1, dist2);
    //  esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], packet_length, packet, false); // change sizes[i] and string[i]
    return ESP_OK;
}
