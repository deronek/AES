#include "mpu9255.h"

// global variables
QueueHandle_t mpu9255_queue_fifo_data;
unsigned long timestamp;

// local variables

// sensor data, not exposed
static mpu9255_fifo_data_type mpu9255_fifo_data;

static const char *TAG = "mpu9255";

static const unsigned char SENSORS = INV_XYZ_GYRO | INV_XYZ_ACCEL;

// constants
#define DEFAULT_MPU_HZ 10
#define TASK_TICK_PERIOD TASK_HZ_TO_TICKS(DEFAULT_MPU_HZ)
#define CALIBRATION_DATA_POINTS 10

// function declarations

static void tap_cb(unsigned char, unsigned char);
static void mpu9255_calibrate();

// function definitions

static void tap_cb(unsigned char a, unsigned char b)
{
}

void mpu9255_calibrate()
{
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

    /*
        // create semaphore used as a data ready signal
        mpu9255_semaphore_fifo_data_ready = xSemaphoreCreateBinary();
        if (mpu9255_semaphore_fifo_data_ready == NULL)
        {
            abort();
        }
    */
    // create queue
    if ((mpu9255_queue_fifo_data = xQueueCreate(1, sizeof(mpu9255_fifo_data_type))) == NULL)
    {
        abort();
    }

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
        retval = dmp_read_fifo(
            mpu9255_fifo_data.gyro_raw.array,
            mpu9255_fifo_data.accel_raw.array,
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
        /*
                // data is ready, take semaphore
                if (xSemaphoreTake(mpu9255_semaphore_fifo_data_ready, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG, "Data was not read");
                }
        */
        xQueueOverwrite(mpu9255_queue_fifo_data, &mpu9255_fifo_data);
        algo_task_notify(TASK_FLAG_MPU9255);
        ble_task_notify(TASK_FLAG_MPU9255);
        task_utils_sleep_or_warning(&last_wake_time, TASK_TICK_PERIOD, TAG);
    }

    vTaskDelete(NULL);

    // should never reach there
    abort();
}