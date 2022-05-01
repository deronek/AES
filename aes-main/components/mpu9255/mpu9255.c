#include "mpu9255.h"

// global variables
QueueHandle_t mpu9255_queue_fifo_data;
unsigned long timestamp;

// local variables

static mpu9255_sensor_data_type gyro_offset;
static mpu9255_sensor_data_type accel_offset;

// sensor data, not exposed
static mpu9255_fifo_data_type mpu9255_fifo_data;

static const char *TAG = "mpu9255";

static const unsigned char SENSORS = INV_XYZ_GYRO | INV_XYZ_ACCEL;

// constants
#define DEFAULT_MPU_HZ 10
#define TASK_TICK_PERIOD TASK_HZ_TO_TICKS(DEFAULT_MPU_HZ)

/**
 * @brief Maximum number of ticks to wait for the next FIFO data.
 * If this will be exceeded, a warning will be logged.
 */
#define TASK_MAXIMUM_TICK_PERIOD TASK_TICK_PERIOD + 2

#define CALIBRATION_DATA_POINTS_EXP 4
#define CALIBRATION_DATA_POINTS (1 << CALIBRATION_DATA_POINTS_EXP)

// function declarations

static void tap_cb(unsigned char, unsigned char);
static void mpu9255_init_int_pin();

// function definitions

void tap_cb(unsigned char a, unsigned char b)
{
}

// void mpu9255_calibrate()
// {
//     mpu9255_fifo_data_type fifo_data;

//     // clear current offset values
//     memset(gyro_offset.array, 0, sizeof(gyro_offset));
//     memcpy(accel_offset.array, 0, sizeof(accel_offset));

//     for (int i = 0; i < CALIBRATION_DATA_POINTS; ++i)
//     {
//         xQueueReceive(mpu9255_queue_fifo_data, &fifo_data, portMAX_DELAY);

//         for (int axis = 0; axis < MPU9255_AXIS_NUMBER; ++axis)
//         {
//             gyro_offset.array[axis] += fifo_data.gyro.array[axis] << CALIBRATION_DATA_POINTS_EXP;
//             accel_offset.array[axis] += fifo_data.accel.array[axis] << CALIBRATION_DATA_POINTS_EXP;
//         }
//     }
// }

void mpu9255_calibrate()
{
    ESP_LOGI(TAG, "Starting calibration, %d points", CALIBRATION_DATA_POINTS);
    long gyro_bias[3] = {0};
    long accel_bias[3] = {0};
    mpu9255_fifo_data_type fifo_data;

    for (int i = 0; i < CALIBRATION_DATA_POINTS; ++i)
    {
        xQueueReceive(mpu9255_queue_fifo_data, &fifo_data, portMAX_DELAY);

        gyro_bias[0] += (long)fifo_data.gyro.x;
        gyro_bias[1] += (long)fifo_data.gyro.y;
        gyro_bias[2] += (long)fifo_data.gyro.z;

        accel_bias[0] += (long)fifo_data.accel.x;
        accel_bias[1] += (long)fifo_data.accel.y;
        accel_bias[2] += (long)fifo_data.accel.z;
    }

    float gyro_sens;
    mpu_get_gyro_sens(&gyro_sens);

    unsigned short accel_sens;
    mpu_get_accel_sens(&accel_sens);

    for (int axis = 0; axis < 3; ++axis)
    {
        // scale sum to get the average measurement
        gyro_bias[axis] /= CALIBRATION_DATA_POINTS;
        accel_bias[axis] /= CALIBRATION_DATA_POINTS;

        // gyro - bias values need to be in +- 1000 dps, scale from +- 2000 dps
        gyro_bias[axis] *= 2;

        // accel - bias values need to be in +- 16G, scale from +- 2G
        accel_bias[axis] /= 8;

        // gyro_bias[axis] /= gyro_sens;
        // accel_bias[axis] /= accel_sens;
    }

    ESP_LOGI(TAG, "Gyro bias: %li %li %li", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    ESP_LOGI(TAG, "Accel bias: %li %li %li", accel_bias[0], accel_bias[1], accel_bias[2]);

    mpu_set_gyro_bias_reg(gyro_bias);
    mpu_set_accel_bias_6500_reg(accel_bias);
}

esp_err_t mpu9255_init()
{
    ESP_ERROR_CHECK(mpu_init(NULL));
    ESP_ERROR_CHECK(mpu_set_sensors(SENSORS));
    ESP_ERROR_CHECK(mpu_configure_fifo(SENSORS));
    ESP_ERROR_CHECK(mpu_set_sample_rate(DEFAULT_MPU_HZ));

    ESP_LOGV(TAG, "Start loading MotionDriver firmware");
    ESP_ERROR_CHECK(dmp_load_motion_driver_firmware());
    ESP_LOGV(TAG, "MotionDriver firmware loaded successfully");

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
    ESP_ERROR_CHECK(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO));

    // register empty tap callback
    ESP_ERROR_CHECK(dmp_register_tap_cb(tap_cb));
    ESP_ERROR_CHECK(dmp_set_fifo_rate(DEFAULT_MPU_HZ));
    ESP_ERROR_CHECK(mpu_set_dmp_state(1));

    ESP_LOGI(TAG, "MPU9255 initialized");

    long gyro[3], accel[3];
    int retval = mpu_run_6500_self_test(gyro, accel, 1);

    if (retval & INV_SELF_TEST_GYRO_PASS)
    {
        ESP_LOGI(TAG, "Gyro self test passed");
    }
    else
    {
        ESP_LOGE(TAG, "Gyro self test failed");
    }

    if (retval & INV_SELF_TEST_ACCEL_PASS)
    {
        ESP_LOGI(TAG, "Accel self test passed");
    }
    else
    {
        ESP_LOGE(TAG, "Accel self test failed");
    }

    if (retval & INV_SELF_TEST_COMPASS_PASS)
    {
        ESP_LOGI(TAG, "Compass self test passed");
    }
    else
    {
        ESP_LOGE(TAG, "Compass self test failed");
    }

    if (retval != INV_SELF_TEST_ALL_PASS)
    {
        abort();
    }

    // create queue
    if ((mpu9255_queue_fifo_data = xQueueCreate(1, sizeof(mpu9255_fifo_data_type))) == NULL)
    {
        abort();
    }

    mpu9255_init_int_pin();
    dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);

    return ESP_OK;
}

TASK mpu9255_task_measure()
{
    short sensors;
    unsigned char more;
    int retval;
    TickType_t last_wake_time;

    last_wake_time = xTaskGetTickCount();
    for (;;)
    {
        uint32_t notification_value = ulTaskNotifyTake(pdTRUE, TASK_MAXIMUM_TICK_PERIOD);
        if (!notification_value)
        {
            ESP_LOGW(TAG, "Maximum tick period exceeded");
            continue;
        }
        retval = dmp_read_fifo(
            mpu9255_fifo_data.gyro.array,
            mpu9255_fifo_data.accel.array,
            mpu9255_fifo_data.quaternion.array,
            &timestamp, &sensors, &more);
        if (retval == -2)
        {
            ESP_LOGW(TAG, "FIFO overflowed, measurement skipped");
        }
        else
        {
            ESP_LOGV(TAG, "Read FIFO data from MPU9255. Timestamp: %lu, bytes left in FIFO: %hhu", timestamp, more);
        }

        // for (int axis = 0; axis < MPU9255_AXIS_NUMBER; ++axis)
        // {
        //     mpu9255_fifo_data.gyro.array[axis] -= gyro_offset.array[axis];
        //     mpu9255_fifo_data.accel.array[axis] -= accel_offset.array[axis];
        // }

        xQueueOverwrite(mpu9255_queue_fifo_data, &mpu9255_fifo_data);
        app_manager_algo_task_notify(TASK_FLAG_MPU9255);
        app_manager_ble_task_notify(TASK_FLAG_MPU9255);

        /**
         * @todo Implement DMP interrupt - we can be notified when the
         * new FIFO data is ready.
         * Use dmp_set_interrupt_mode(DMP_INT_CONTINUOUS) and INT pin.
         */
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

void mpu9255_init_int_pin()
{
    gpio_reset_pin(MPU9255_INT_PIN);
    gpio_set_direction(MPU9255_INT_PIN, GPIO_MODE_INPUT);
    gpio_set_intr_type(MPU9255_INT_PIN, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(MPU9255_INT_PIN, mpu9255_isr, (void *)MPU9255_INT_PIN);
}