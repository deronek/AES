#include "mpu9255.h"

#include "driver/i2c.h"
#include "freertos/semphr.h"

#include "app_manager.h"
#include "i2c.h"

#include <string.h>

// global variables
QueueHandle_t mpu9255_queue_quaternion_data;
QueueHandle_t mpu9255_queue_gyro_data;
QueueHandle_t mpu9255_queue_accel_data;
unsigned long timestamp;

// local variables
static mpu9255_sensor_data_type mag_bias;
static bool mpu9255_ble_sending = false;

// sensor data, not exposed
static mpu9255_quaternion_data_type mpu9255_quaternion_data;
static mpu9255_sensor_data_type mpu9255_gyro_data;
static mpu9255_sensor_data_type mpu9255_accel_data;
static mpu9255_sensor_data_type mpu9255_mag_data;

static const char *TAG = "mpu9255";

static const unsigned char SENSORS = INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS;

// constants
#define MPU_SAMPLE_RATE 20
#define TASK_TICK_PERIOD TASK_HZ_TO_TICKS(MPU_SAMPLE_RATE)
#define DMP_FLASH_TRIES (5)

/**
 * @brief Maximum number of ticks to wait for the next FIFO data.
 * If this will be exceeded, a warning will be logged.
 */
#define TASK_MAXIMUM_TICK_PERIOD TASK_TICK_PERIOD + 2

#define CALIBRATION_DATA_POINTS_EXP 6
#define CALIBRATION_DATA_POINTS (1 << CALIBRATION_DATA_POINTS_EXP)

// function declarations

static void tap_cb(unsigned char, unsigned char);

// function definitions

void tap_cb(unsigned char a, unsigned char b)
{
}

static long gyro_bias[3] = {0};
static long accel_bias[3] = {0};

void mpu9255_get_calibration()
{
    ESP_LOGI(TAG, "Starting calibration, %d points", CALIBRATION_DATA_POINTS);
    mpu9255_sensor_data_type gyro_data, accel_data;

    memset(gyro_bias, 0, sizeof(gyro_bias));
    memset(accel_bias, 0, sizeof(accel_bias));

    memset(&gyro_data, 0, sizeof(gyro_data));
    memset(&accel_data, 0, sizeof(accel_data));

    short gyro_measurement[3], accel_measurement[3];

    for (int i = 0; i < CALIBRATION_DATA_POINTS; ++i)
    {
        xQueueReceive(mpu9255_queue_gyro_data, &gyro_data, portMAX_DELAY);
        xQueueReceive(mpu9255_queue_accel_data, &accel_data, portMAX_DELAY);

        gyro_bias[0] += (long)gyro_data.x;
        gyro_bias[1] += (long)gyro_data.y;
        gyro_bias[2] += (long)gyro_data.z;

        accel_bias[0] += (long)accel_data.x;
        accel_bias[1] += (long)accel_data.y;
        accel_bias[2] += (long)accel_data.z;
    }

    for (int axis = 0; axis < 3; ++axis)
    {
        // gyro - bias values need to be in +- 1000 dps, scale from +- 2000 dps
        float gyro_bias_f = roundf((float)gyro_bias[axis] / CALIBRATION_DATA_POINTS * 2);

        // accel - bias values need to be in +- 16G, scale from +- 2G
        float accel_bias_f = roundf((float)accel_bias[axis] / CALIBRATION_DATA_POINTS / 8);

        gyro_bias[axis] = gyro_bias_f;
        accel_bias[axis] = accel_bias_f;
    }

    ESP_LOGI(TAG, "Measured gyro bias: %li %li %li", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    ESP_LOGI(TAG, "Measured accel bias: %li %li %li", accel_bias[0], accel_bias[1], accel_bias[2]);

    /**
     * @brief Read bias register values.
     */
    long gyro_read[3], accel_read[3];

    mpu_read_6500_gyro_bias(gyro_read);
    mpu_read_6500_accel_bias(accel_read);

    ESP_LOGI(TAG, "Read gyro bias: %li %li %li", gyro_read[0], gyro_read[1], gyro_read[2]);
    ESP_LOGI(TAG, "Read accel bias: %li %li %li", accel_read[0], accel_read[1], accel_read[2]);

    /**
     * @brief Apply correction to read biases.
     */

    gyro_bias[0] = gyro_read[0] - gyro_bias[0];
    gyro_bias[1] = gyro_read[1] - gyro_bias[1];
    gyro_bias[2] = gyro_read[2] - gyro_bias[2];

    // do not change the temperature compensation bit
    accel_bias[0] = accel_read[0] - (accel_bias[0] & ~1);
    accel_bias[1] = accel_read[1] - (accel_bias[1] & ~1);
    accel_bias[2] = accel_read[2] - (accel_bias[2] & ~1);

    ESP_LOGI(TAG, "Resulting gyro bias: %li %li %li", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    ESP_LOGI(TAG, "Resulting accel bias: %li %li %li", accel_bias[0], accel_bias[1], accel_bias[2]);
}

esp_err_t mpu9255_apply_calibration()
{
    esp_err_t gyro_retval = mpu_set_gyro_bias_reg_direct(gyro_bias);
    esp_err_t accel_retval = mpu_set_accel_bias_6500_reg_direct(accel_bias);

    return ESP_OK ? (gyro_retval | accel_retval) == ESP_OK : ESP_FAIL;
}

esp_err_t mpu9255_init()
{
    gpio_reset_pin(MPU9255_INT_PIN);
    gpio_set_direction(MPU9255_INT_PIN, GPIO_MODE_INPUT);
    gpio_set_intr_type(MPU9255_INT_PIN, GPIO_INTR_POSEDGE);

    mpu9255_reset();
    // unsigned short rate;
    // mpu_get_compass_sample_rate(&rate);
    // ESP_LOGI(TAG, "Compass sample rate: %hu", rate);

    // ESP_LOGI(TAG, "MPU9255 initialized");

    // long gyro[3], accel[3];
    // int retval = mpu_run_6500_self_test(gyro, accel, 1);

    // if (retval & INV_SELF_TEST_GYRO_PASS)
    // {
    //     ESP_LOGI(TAG, "Gyro self test passed");
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "Gyro self test failed");
    // }

    // if (retval & INV_SELF_TEST_ACCEL_PASS)
    // {
    //     ESP_LOGI(TAG, "Accel self test passed");
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "Accel self test failed");
    // }

    // if (retval & INV_SELF_TEST_COMPASS_PASS)
    // {
    //     ESP_LOGI(TAG, "Compass self test passed");
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "Compass self test failed");
    // }

    // if (retval != INV_SELF_TEST_ALL_PASS)
    // {
    //     abort();
    // }

    // create queue
    if ((mpu9255_queue_quaternion_data = xQueueCreate(1, sizeof(mpu9255_quaternion_data_type))) == NULL)
    {
        abort();
    }
    if ((mpu9255_queue_gyro_data = xQueueCreate(1, sizeof(mpu9255_sensor_data_type))) == NULL)
    {
        abort();
    }
    if ((mpu9255_queue_accel_data = xQueueCreate(10, sizeof(mpu9255_sensor_data_type))) == NULL)
    {
        abort();
    }

    return ESP_OK;
}

void mpu9255_reset()
{
    ESP_ERROR_CHECK(mpu9255_set_isr(false));

    ESP_ERROR_CHECK(mpu_init(NULL));
    ESP_ERROR_CHECK(mpu_set_sensors(SENSORS));
    ESP_ERROR_CHECK(mpu_configure_fifo(SENSORS));

    /**
     * @brief Loading MotionDriver firmware randomly crashes when it's not the first time
     * loading it. Sometimes the chip does not respond for writing, for reading, or
     * the packet can be "received" by the chip but not actually written to memory.
     * Disabling DMP/features or resetting program start address did not resolve the issue.
     *
     * @todo It would be good to repeat single packets when it's necessary,
     * not try to write the whole firmware again.
     */
    ESP_LOGV(TAG, "Start loading MotionDriver firmware");
    int retval;
    for (int i = 0; i < DMP_FLASH_TRIES; ++i)
    {
        retval = dmp_load_motion_driver_firmware();
        if (retval == 0)
        {
            break;
        }
    }
    if (retval != 0)
    {
        abort();
    }
    ESP_LOGV(TAG, "MotionDriver firmware loaded successfully");
    ESP_ERROR_CHECK(mpu_set_sample_rate(MPU_SAMPLE_RATE));

    // set default orientation - X, Y, Z
    ESP_ERROR_CHECK(dmp_set_orientation(0b010001000));

    // enable 6-axis quaternion, raw accel and raw gyro
    // tap feature is enabled beacuse of the bug
    /*
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     *
     * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
     */
    ESP_ERROR_CHECK(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO | DMP_FEATURE_GYRO_CAL));

    // register empty tap callback
    ESP_ERROR_CHECK(dmp_register_tap_cb(tap_cb));
    ESP_ERROR_CHECK(dmp_set_fifo_rate(MPU_SAMPLE_RATE));
    ESP_ERROR_CHECK(mpu_set_dmp_state(1));
    // ESP_ERROR_CHECK(mpu_set_compass_sample_rate(MPU_SAMPLE_RATE));
    dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
}

TASK mpu9255_task_measure()
{
    mpu_reset_fifo();
    short sensors;
    unsigned char more;
    int retval;
    TickType_t last_wake_time;
    short status;

    last_wake_time = xTaskGetTickCount();
    for (;;)
    {
        uint32_t notification_value = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // if (!notification_value)
        // {
        //     ESP_LOGW(TAG, "Maximum tick period exceeded");
        //     continue;
        // }
        // ESP_ERROR_CHECK(mpu_get_int_status(&status));
        // ESP_LOGI(TAG, "DMP interrupt before: %hx", status);
        retval = dmp_read_fifo(
            mpu9255_gyro_data.array,
            mpu9255_accel_data.array,
            mpu9255_quaternion_data.array,
            &timestamp, &sensors, &more);
        if (retval == -2)
        {
            ESP_LOGW(TAG, "FIFO overflowed, measurement skipped");
        }
        else
        {
            ESP_LOGV(TAG, "Read FIFO data from MPU9255. Timestamp: %lu, bytes left in FIFO: %hhu", timestamp, more);
        }

        mpu_get_compass_reg(mpu9255_mag_data.array, NULL);

        for (int axis = 0; axis < MPU9255_AXIS_NUMBER; ++axis)
        {
            // mpu9255_quaternion_data.gyro.array[axis] -= gyro_offset.array[axis];
            // mpu9255_quaternion_data.accel.array[axis] -= accel_offset.array[axis];
            // mpu9255_quaternion_data.mag.array[axis] -= mag_bias.array[axis];
        }
        // ESP_ERROR_CHECK(mpu_get_int_status(&status));
        // ESP_LOGI(TAG, "DMP interrupt after: %hx", status);

        xQueueOverwrite(mpu9255_queue_quaternion_data, &mpu9255_quaternion_data);
        xQueueOverwrite(mpu9255_queue_gyro_data, &mpu9255_gyro_data);
        int retval = xQueueSend(mpu9255_queue_accel_data, &mpu9255_accel_data, 0);
        if (retval == errQUEUE_FULL)
        {
            // ESP_LOGE(TAG, "Accel queue data full, sample lost");
        }

        // app_manager_algo_task_notify(TASK_FLAG_MPU9255);

        // if (mpu9255_ble_sending)
        // {
        //     app_manager_ble_task_notify(TASK_FLAG_MPU9255);
        // }

        // task_utils_sleep_or_warning(&last_wake_time, TASK_TICK_PERIOD, TAG);
    }

    vTaskDelete(NULL);
    // should never reach there
    abort();
}

static void IRAM_ATTR mpu9255_isr(void *arg)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(app_manager_mpu9255_task_handle, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

esp_err_t mpu9255_set_isr(bool enabled)
{
    if (enabled)
    {
        // mpu9255_running = true;
        return gpio_isr_handler_add(MPU9255_INT_PIN, mpu9255_isr, NULL);
    }
    else
    {
        // mpu9255_running = false;
        return gpio_isr_handler_remove(MPU9255_INT_PIN);
    }
}

void mpu9255_set_ble_sending(bool enabled)
{
    mpu9255_ble_sending = enabled;
}
