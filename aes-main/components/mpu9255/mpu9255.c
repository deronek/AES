#include "mpu9255.h"

// global variables
short gyro_raw[3], accel_raw[3];
long quat[4];
unsigned long timestamp;

// local variables

static const char *TAG = "mpu9255";

static const unsigned char SENSORS = INV_XYZ_GYRO | INV_XYZ_ACCEL;

// constants
#define DEFAULT_MPU_HZ          10
#define MPU9255_TASK_TICK_FREQ  (1.0f / DEFAULT_MPU_HZ) * configTICK_RATE_HZ

// function declarations

static void tap_cb(unsigned char, unsigned char);

// function definitions

static void tap_cb(unsigned char a, unsigned char b)
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

    ESP_LOGV(TAG, "Task tick is %f btw", MPU9255_TASK_TICK_FREQ);

    return ESP_OK;
}

void mpu9255_task_measure()
{
    short sensors;
    unsigned char more;
    unsigned char retval;
    TickType_t last_wake_time;

    last_wake_time = xTaskGetTickCount();
    for(;;)
    {
        retval = dmp_read_fifo(gyro_raw, accel_raw, quat, &timestamp, &sensors, &more);
        if (retval == -2)
        {
            ESP_LOGW(TAG, "FIFO overflowed, measurement skipped");
        }
        else 
        {
            ESP_LOGV(TAG, "Read FIFO data from MPU9255. Timestamp: %lu, bytes left in FIFO: %hhu", timestamp, more);
        }

        sleep_or_warning(&last_wake_time, MPU9255_TASK_TICK_FREQ, TAG);
    }
}