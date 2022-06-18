#include "position.h"

#include "heading_imu.h"
#include "photo_encoder.h"
#include "mpu9255.h"
#include "hp_iir_filter.h"
#include <math.h>

// structs
typedef struct accel_velocity_type_tag
{
    float x;
    float y;
} accel_velocity_type;

// constants
#define POSITION_COMP_FILTER_ALPHA (0.5)
#define G_UNIT_TO_M_S2 (9.80665)
#define ACCEL_LP_FILTER_ALPHA (0.5)
#define TIME_DELTA_ACCEL (1.0 / 200)

#define ALGO_POSITION_FREQUENCY (10)
#define ALGO_POSITION_DELTA_TIME (1 / ALGO_POSITION_FREQUENCY)
#define TASK_TICK_PERIOD (TASK_HZ_TO_TICKS(ALGO_POSITION_FREQUENCY))

#define PHOTO_ENCODER_TIMEOUT_MS (50)
#define PHOTO_ENCODER_TIMEOUT_TICKS (pdMS_TO_TICKS(PHOTO_ENCODER_TIMEOUT_MS))

#define ACCEL_TIMEOUT_MS (10)
#define ACCEL_TIMEOUT_TICKS (pdMS_TO_TICKS(ACCEL_TIMEOUT_MS))

// global variables
QueueHandle_t position_queue;

// local variables
static QueueHandle_t photo_encoder_position_queue;
static QueueHandle_t accel_velocity_queue;
static mpu9255_sensor_data_type accel_data;

static hp_iir_filter_type *accel_lp_filter_x;
static hp_iir_filter_type *accel_lp_filter_y;

static const char *TAG = "algo-position";

static bool position_process_stop_requested = false;
static bool photo_encoder_process_stop_requested = false;
static bool accel_process_stop_requested = false;

void position_init()
{
    // initialize complementary filter
    // filter = comp_iir_filter_init(COMP_FILTER_ALPHA);

    // initialize queue for velocity data from acceleromter
    accel_velocity_queue = xQueueCreate(1, sizeof(accel_velocity_type));
    if (accel_velocity_queue == NULL)
    {
        abort();
    }

    // initialize queue for position data from wheel encoders
    photo_encoder_position_queue = xQueueCreate(1, sizeof(photo_encoder_position_type));
    if (photo_encoder_position_queue == NULL)
    {
        abort();
    }

    // initialize queue for total calculated position data
    position_queue = xQueueCreate(1, sizeof(photo_encoder_position_type));
    if (position_queue == NULL)
    {
        abort();
    }

    // initialize LP IIR filter for accel velocity data
    accel_lp_filter_x = hp_iir_filter_init(ACCEL_LP_FILTER_ALPHA);
    accel_lp_filter_y = hp_iir_filter_init(ACCEL_LP_FILTER_ALPHA);
}

TASK position_process()
{
    int retval;
    algo_position_type position = {0};

    /**
     * @brief Put (0, 0) position into position_queue
     * so that the data can be used in the algorithm.
     */
    xQueueOverwrite(position_queue, &position);

    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;)
    {
        // ESP_LOGI(TAG, "Position start iter");
        if (position_process_stop_requested)
        {
            break;
        }
        accel_velocity_type accel_velocity;
        retval = xQueuePeek(accel_velocity_queue, &accel_velocity, 0);
        if (retval != pdTRUE)
        {
            ESP_LOGE(TAG, "No data in accel queue");
        }

        photo_encoder_position_type photo_encoder_position;
        retval = xQueuePeek(photo_encoder_position_queue, &photo_encoder_position, 0);
        if (retval != pdTRUE)
        {
            ESP_LOGE(TAG, "No data found in photo encoder queue");
        }

        /**
         * @brief Multiplying velocity by delta time gives
         * position change from the accelerometer in that time.
         *
         * @todo Check whether this task should run at faster frequency
         * than total of the algo task; it might result in more accurate
         * position estimate.
         */
        float accel_delta_x = accel_velocity.x * ALGO_POSITION_DELTA_TIME;
        float accel_delta_y = accel_velocity.y * ALGO_POSITION_DELTA_TIME;

        /**
         * @brief Get accelerometer position estimate by adding
         * total previous estimate to the current change.
         */
        float accel_x = accel_delta_x + position.x;
        float accel_y = accel_delta_y + position.y;

        /**
         * @brief Update position variable.
         */
        position.x = (POSITION_COMP_FILTER_ALPHA * accel_x) +
                     ((1 - POSITION_COMP_FILTER_ALPHA) * photo_encoder_position.x);

        position.y = (POSITION_COMP_FILTER_ALPHA * accel_y) +
                     ((1 - POSITION_COMP_FILTER_ALPHA) * photo_encoder_position.y);

        xQueueOverwriteFromISR(position_queue, &position, NULL);
        // xQueueOverwrite(position_queue, &position);
        // ESP_LOGI(TAG, "Position stop iter");
        task_utils_sleep_or_warning(&last_wake_time, TASK_TICK_PERIOD, TAG);
    }
    xQueueReset(position_queue);

    ESP_LOGI(TAG, "Suspending position_process");
    xTaskNotifyGive(app_manager_algo_task_handle);
    vTaskSuspend(NULL);
}

TASK position_photo_encoder_process()
{
    BaseType_t retval;
    float delta_left = 0;
    float delta_right = 0;

    photo_encoder_position_type pos = {0};
    /**
     * @brief Put (0, 0) position into photo_encoder_position_queue
     * so that the data can be used in the algorithm.
     */
    xQueueOverwrite(photo_encoder_position_queue, &pos);

    for (;;)
    {
        if (photo_encoder_process_stop_requested)
        {
            break;
        }
        uint32_t notify_val = ulTaskNotifyTake(pdTRUE, PHOTO_ENCODER_TIMEOUT_TICKS);
        if (notify_val == 0)
        {
            /**
             * @brief We did not get a tick from photo encoders in
             * PHOTO_ENCODER_TIMEOUT_MS.
             * Signal a warning.
             */
            // ESP_LOGW(TAG, "Photo encoder receive timeout");
            continue;
        }

        /**
         * @brief Check notify value from photo encoder ISR.
         */
        if (notify_val | PHOTO_ENCODER_L_WIDTH)
        {
            delta_left += WIDTH_STRIPE_M;
        }
        if (notify_val | PHOTO_ENCODER_L_DISTANCE)
        {
            delta_left += DISTANCE_STRIPE_M;
        }
        if (notify_val | PHOTO_ENCODER_R_WIDTH)
        {
            delta_right += WIDTH_STRIPE_M;
        }
        if (notify_val | PHOTO_ENCODER_R_DISTANCE)
        {
            delta_right += DISTANCE_STRIPE_M;
        }

        float delta_center = (delta_left + delta_right) / 2;

        /**
         * @brief Read current heading. This read is atomic.
         * @todo It might be possible to use photo encoder data
         * to also get heading change. Maybe somehow use this
         * to make the position measurement more accurate.
         */
        volatile float heading = algo_current_heading;

        /**
         * @todo Try to use sincosf instead here, might be faster.
         */
        pos.x += delta_center * cosf(heading);
        pos.y += delta_center * sinf(heading);

        xQueueOverwriteFromISR(photo_encoder_position_queue, &pos, NULL);
        // xQueueOverwrite(photo_encoder_position_queue, &pos);

        /**
         * @brief Reset delta values to zero.
         * @todo Might be possible to do some linear interpolation here instead
         * of only using current value.
         */
        delta_left = 0;
        delta_right = 0;
    }
    xQueueReset(photo_encoder_position_queue);

    ESP_LOGI(TAG, "Suspending position_photo_encoder_process");
    xTaskNotifyGive(app_manager_algo_task_handle);
    vTaskSuspend(NULL);
}

TASK position_accel_process()
{
    BaseType_t retval;
    /**
     * @todo At first iteration, integration value should be identical to rectangle.
     * (x_accel_n_1 = x_accel etc.)
     * This is not a problem if we let the first iteration run at standstill
     * (then x_accel_n_1 ≈ x_accel ≈ 0).
     */
    float x_accel_n_1 = 0;
    float y_accel_n_1 = 0;

    /**
     * @brief Put (0, 0) velocity in accel_velocity_queue
     * so that the data can be used in the algorithm.
     */
    accel_velocity_type v = {0};
    xQueueOverwrite(accel_velocity_queue, &v);
    for (;;)
    {
        if (accel_process_stop_requested)
        {
            break;
        }
        retval = xQueueReceive(mpu9255_queue_accel_data, &accel_data, ACCEL_TIMEOUT_TICKS);
        if (retval != pdTRUE)
        {
            ESP_LOGE(TAG, "Accel receive timeout");
            continue;
        }

        float x_accel = (float)accel_data.x * G_UNIT_TO_M_S2 * 2 / INT16_MAX;
        float y_accel = (float)accel_data.y * G_UNIT_TO_M_S2 * 2 / INT16_MAX;

#if defined(POSITION_RECTANGLE)
        v.x += (x_accel * TIME_DELTA_ACCEL);
        v.y += (y_accel * TIME_DELTA_ACCEL);
#elif defined(POSITION_TRAPEZOIDAL)

        v.x += ((x_accel + x_accel_n_1) / 2 * TIME_DELTA_ACCEL);
        v.y += ((y_accel + y_accel_n_1) / 2 * TIME_DELTA_ACCEL);

        // save n-1 values for next iteration
        x_accel_n_1 = x_accel;
        y_accel_n_1 = y_accel;
#else
#error "Define method for accelerometer position integration."
#endif

        /**
         * @brief Apply high-pass filter on velocity data.
         * We only need short-term change of it and we want
         * to kill any build-up offset.
         */
        v.x = hp_iir_filter_step(accel_lp_filter_x, v.x);
        v.y = hp_iir_filter_step(accel_lp_filter_x, v.y);

        // ESP_LOGI(TAG, "x_vel = %.2f", x_vel);
        // ESP_LOGI(TAG, "y_vel = %.2f", y_vel);
        xQueueOverwriteFromISR(accel_velocity_queue, &v, NULL);
        // xQueueOverwrite(accel_velocity_queue, &v);
    }
    xQueueReset(accel_velocity_queue);

    ESP_LOGI(TAG, "Suspending position_accel_process");
    xTaskNotifyGive(app_manager_algo_task_handle);
    vTaskSuspend(NULL);
}

/**
 * @brief Reset all of the calculations and data structures.
 * Must be called when process tasks are NOT running.
 */
void position_reset()
{
    position_process_stop_requested = false;
    photo_encoder_process_stop_requested = false;
    accel_process_stop_requested = false;
}

void position_process_request_stop()
{
    position_process_stop_requested = true;
}

void position_photo_encoder_process_request_stop()
{
    photo_encoder_process_stop_requested = true;
}

void position_accel_process_request_stop()
{
    accel_process_stop_requested = true;
}