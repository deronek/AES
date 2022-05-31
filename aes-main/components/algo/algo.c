#include "algo.h"

#include "data_receive.h"
#include "photo_encoder.h"
#include "hall.h"

// constants
#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)

#define COMP_FILTER_ALPHA 0.5

// global variables
algo_quaternion_type algo_quaternion;
algo_euler_angles_type algo_euler_angles;
QueueHandle_t algo_heading_data_queue;

/**
 * @todo Refactor this with getter
 */
bool algo_running = false;

// local variables
/**
 * @brief Heading of finish line in degress;
 * @todo Make this value dynamic from controller app.
 */
static const char *TAG = "algo";

static float finish_heading = 30.0;

float phiHat_rad = 0.0f;
float thetaHat_rad = 0.0f;

// constants
#define ALGO_FREQUENCY 10
#define TASK_TICK_PERIOD TASK_HZ_TO_TICKS(ALGO_FREQUENCY)

#define QUATERNION_SCALE_FACTOR (2 << 29)

// function declarations
static void algo_update_quaternion();
static float algo_low_pass_tick(float xn);
static float algo_high_pass_tick(float xn);

// function definitions
void algo_init()
{
    // hall_init();
    /**
     * @todo Initialize all algo data structures
     */
    if ((algo_heading_data_queue = xQueueCreate(1, sizeof(algo_heading_data_type))) == NULL)
    {
        abort();
    }
}

void algo_update_quaternion()
{
    algo_quaternion.w = (float)algo_mpu9255_fifo_data.quaternion.w / QUATERNION_SCALE_FACTOR;
    algo_quaternion.x = (float)algo_mpu9255_fifo_data.quaternion.x / QUATERNION_SCALE_FACTOR;
    algo_quaternion.y = (float)algo_mpu9255_fifo_data.quaternion.y / QUATERNION_SCALE_FACTOR;
    algo_quaternion.z = (float)algo_mpu9255_fifo_data.quaternion.z / QUATERNION_SCALE_FACTOR;
}

TASK algo_main()
{
    algo_running = true;
    // photo_encoder_enable_isr();
    // get start time of this iteration
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;)
    {
        // wait for notification, which will be sent by any of the sensor tasks
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /**
         * @todo Implement to only get data which actually updated,
         * so we do not have to copy every one of it.
         * Can probably implemented using bit flags in task notification value.
         */
        // update algo data from sensors
        algo_data_receive_get_latest();

        // run algo calculations
        // heading_calculate();

        algo_run();

        /**
         * @brief We want to run algo calculations at least with ALGO_FREQUENCY.
         * If we do not meet that time, signal a warning.
         */
        // task_utils_sleep_or_warning(&last_wake_time, TASK_TICK_PERIOD, TAG);
    }
}

void algo_run()
{
    // float gyro_sens;
    // mpu_get_gyro_sens(&gyro_sens);

    // unsigned short accel_sens;
    // mpu_get_accel_sens(&accel_sens);

    // float gyro_x = (float)algo_mpu9255_fifo_data.gyro.x / gyro_sens;
    // float gyro_y = (float)algo_mpu9255_fifo_data.gyro.y / gyro_sens;
    // float gyro_z = (float)algo_mpu9255_fifo_data.gyro.z / gyro_sens;

    // float accel_x = (float)algo_mpu9255_fifo_data.accel.x / accel_sens;
    // float accel_y = (float)algo_mpu9255_fifo_data.accel.y / accel_sens;
    // float accel_z = (float)algo_mpu9255_fifo_data.accel.z / accel_sens;

    // CompGyroUpdate(&filter, gyro_x, gyro_y, gyro_z);
    // CompAccelUpdate(&filter, accel_x, accel_y, accel_z);

    // CompUpdate(&filter);

    // float x, y;
    // CompAnglesGet(&filter, &x, &y);

    // ESP_LOGI(TAG, "%.2f %.2f", x * RAD_TO_DEG, y * RAD_TO_DEG);

    // float phiHat_acc_rad = atanf(accel_y / accel_z);
    // float thetaHat_acc_rad = asinf(accel_x / 9.81f);

    // float phiDot_rps = gyro_x + tanf(thetaHat_rad) * (sinf(phiHat_rad) * gyro_y + cosf(phiHat_rad) * gyro_z);
    // float thetaDot_rps = cosf(phiHat_rad) * gyro_y - sinf(phiHat_rad) * gyro_z;

    // // phiHat_rad = phiHat_rad + (0.1f) * phiDot_rps;
    // // thetaHat_rad = thetaHat_rad + (0.1f) * thetaDot_rps;

    // phiHat_rad = COMP_FILTER_ALPHA * phiHat_acc_rad + (1.0f - COMP_FILTER_ALPHA) * (phiHat_rad + (0.1f) * phiDot_rps);

    // thetaHat_rad = COMP_FILTER_ALPHA * thetaHat_acc_rad + (1.0 - COMP_FILTER_ALPHA) * (thetaHat_rad + (0.1f) * thetaDot_rps);

    // ESP_LOGI(TAG, "%.3f %.3f", phiHat_rad * RAD_TO_DEG, thetaHat_rad * RAD_TO_DEG);

    algo_update_quaternion();
    // yaw
    float siny_cosp = 2 * (algo_quaternion.w * algo_quaternion.z +
                           algo_quaternion.x * algo_quaternion.y);
    float cosy_cosp = 1 - 2 * (algo_quaternion.y * algo_quaternion.y +
                               algo_quaternion.z * algo_quaternion.z);
    float yaw_quat = atan2f(siny_cosp, cosy_cosp);

    // algo_heading_data_type algo_heading;
    // algo_heading.heading = (int16_t)(algo_euler_angles.yaw * 10);
    // // ESP_LOGI(TAG, "%d", algo_heading.heading);

    // xQueueOverwrite(algo_heading_data_queue, &algo_heading);
    // app_manager_ble_task_notify(TASK_FLAG_ALGO);

    // // roll
    // float sinr_cosp = 2 * (algo_quaternion.w * algo_quaternion.x + algo_quaternion.y * algo_quaternion.z);
    // float cosr_cosp = 1 - 2 * (algo_quaternion.x * algo_quaternion.x + algo_quaternion.y * algo_quaternion.y);
    // algo_euler_angles.roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    // // pitch
    // float sinp = 2 * (algo_quaternion.w * algo_quaternion.y - algo_quaternion.z * algo_quaternion.x);
    // if (abs(sinp) >= 1)
    // {
    //     ESP_LOGW(TAG, "Pitch out of range");
    //     algo_euler_angles.pitch = copysignf(M_PI / 2, sinp);
    // }
    // else
    // {
    //     algo_euler_angles.pitch = asinf(sinp);
    // }

    float mag_x = algo_mpu9255_fifo_data.mag.x;
    float mag_y = algo_mpu9255_fifo_data.mag.y;
    // float mag_z = algo_mpu9255_fifo_data.mag.z * 0.6;

    float yaw_mag = atan2f(mag_y, mag_x);
    float declination = 8.133;
    yaw_mag += declination;

    float yaw_complimentary = algo_low_pass_tick(yaw_mag) + algo_high_pass_tick(yaw_quat);
    yaw_complimentary *= RAD_TO_DEG;

    ESP_LOGI(TAG, "Yaw: %.2f", yaw_complimentary);

    // float yaw = atanf((-mag_y * cosf(algo_euler_angles.roll) + mag_z * sinf(algo_euler_angles.roll)) / (mag_x * cosf(algo_euler_angles.pitch) + mag_y * sinf(algo_euler_angles.roll) * sinf(algo_euler_angles.pitch) + mag_z * cosf(algo_euler_angles.roll) * sinf(algo_euler_angles.pitch)));

    // ESP_LOGI(TAG, "Yaw: %.2f", algo_euler_angles.yaw);
    // ESP_LOGI(TAG, "Mag deg: %.2f", deg);
    // ESP_LOGI(TAG, "Yaw: %.2f", yaw * RAD_TO_DEG);

    // ESP_LOGI(TAG, "Gyro: %hi %hi %hi", algo_mpu9255_fifo_data.gyro.x, algo_mpu9255_fifo_data.gyro.y, algo_mpu9255_fifo_data.gyro.z);
    // ESP_LOGI(TAG, "Mag: %hi %hi %hi", algo_mpu9255_fifo_data.mag.x, algo_mpu9255_fifo_data.mag.y, algo_mpu9255_fifo_data.mag.z);
}

float algo_high_pass_tick(float xn)
{
    static float yn_1 = 0;
    static float xn_1 = 0;

    float yn = COMP_FILTER_ALPHA * yn_1 + COMP_FILTER_ALPHA * (xn_1 - xn);

    yn_1 = yn;
    xn_1 = xn;

    return yn;
}

float algo_low_pass_tick(float xn)
{
    static float yn_1 = 0;

    float yn = (1 - COMP_FILTER_ALPHA) * xn + COMP_FILTER_ALPHA * yn_1;

    yn_1 = yn;

    return yn;
}