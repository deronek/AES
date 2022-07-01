#include "motor.h"

#include "algo.h"
#include "calc_utils.h"

#include "driver/mcpwm.h"

#include <math.h>

// constants
/**
 * @brief Minimum and maximum vehicle speed, in percent of maximum motors speed.
 * Values from 0 to 100.
 * Minimum vehicle speed should be set as lowest value which do not make motors
 * turn (because of motor/tracks/ground friction).
 */
#define SPEED_MIN (7.0F)
// #define SPEED_MIN (6.5F)
#define SPEED_MID (26.0F)
#define SPEED_MAX (30.0F)

#define MOTOR_CORRECTION_L (1.0F)
#define MOTOR_CORRECTION_R (0.95F)

#define SPEED_MIN_L (SPEED_MIN * MOTOR_CORRECTION_L)
#define SPEED_MIN_R (SPEED_MIN * MOTOR_CORRECTION_R)

#define SPEED_MID_L (SPEED_MID * MOTOR_CORRECTION_L)
#define SPEED_MID_R (SPEED_MID * MOTOR_CORRECTION_R)

#define SPEED_MAX_L (SPEED_MAX * MOTOR_CORRECTION_L)
#define SPEED_MAX_R (SPEED_MAX * MOTOR_CORRECTION_R)

/**
 * @todo Maybe correct speeds on motors each.
 * Vehicle is driving a little bit to the left -
 * correct that by giving left motor a little more speed.
 */

#define PWM_FREQUENCY 5000U

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)

/**
 * @brief This needs to refactored, because we probably will not
 * use finish heading, but end point.
 */
#define FINISH_HEADING (30.0 * DEG_TO_RAD)

#define DIR1_GPIO_NUM GPIO_NUM_18
#define DIR2_GPIO_NUM GPIO_NUM_5
#define PWM1_GPIO_NUM GPIO_NUM_17
#define PWM2_GPIO_NUM GPIO_NUM_16

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)

// /**
//  * @brief Velocity (in m/s) of the robot moving forward when both
//  * motors are at full speed.
//  */
// #define V0 (0.2)

// /**
//  * @brief Motor constants.
//  * @todo Fill in the values.
//  */
// #define L (0.0)
// #define R (0.0)

/**
 * @brief Proportional, derivative and integral part
 * coefficients of the motor PID regulator.
 * @todo Adjust these values.
 */
#define kP (5.0)
#define kD (0.3)
// #define kD (0.5)
#define kI (0.02)
// #define kI (0.00)

/**
 * @brief Derivative term implemented as IIR high-pass filter.
 * Alpha value adjustable below.
 */
#define ERROR_DOT_ALPHA (0.7)

// structs
/**
 * @brief Struct to hold input data for motor control.
 *
 * Examples:
 * - forward movement:
 *      - angle = 0.0f
 * -
 */

typedef struct motor_control_output_data_type_tag
{
    uint8_t dir1;
    uint8_t dir2;
    float pwm1;
    float pwm2;
} motor_control_output_data_type;

// local variables
static const char *TAG = "motor";
static motor_control_input_data_type motor_control_input_data;
static motor_control_output_data_type motor_control_output_data;

/**
 * @brief Variable below should be initialized to desired heading
 * (since we start from heading 0 degrees).
 * This will result in error_dot ≈ 0 in first iteration.
 */
static float old_error = 0.0;
static float old_error_dot = 0.0;
static float error_hat = 0.0;

// local function declarations
static float motor_calculate_pwm1(float omega);
static float motor_calculate_pwm2(float omega);
static void motor_perform_control();

// function definitions
void motor_init()
{
    gpio_reset_pin(DIR1_GPIO_NUM);
    gpio_set_direction(DIR1_GPIO_NUM, GPIO_MODE_OUTPUT);
    gpio_reset_pin(DIR2_GPIO_NUM);
    gpio_set_direction(DIR2_GPIO_NUM, GPIO_MODE_OUTPUT);

    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM1_GPIO_NUM));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, PWM2_GPIO_NUM));

    mcpwm_config_t mcpwm_config1 = {
        .frequency = PWM_FREQUENCY,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER};

    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &mcpwm_config1));
    ESP_ERROR_CHECK(mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A));

    mcpwm_config_t mcpwm_config2 = {
        .frequency = PWM_FREQUENCY,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER};

    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &mcpwm_config2));
    ESP_ERROR_CHECK(mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_A));
}

TASK motor_main()
{
    ;
}

void motor_tick(motor_control_input_data_type input_data)
{
    // /**
    //  * @brief Calculate omega using current_heading, expected_heading
    //  * and a PID regulator.
    //  * Omega needs to be in rad/s.
    //  */
    // float omega = 0.0;

    // float v_left = (2 * SPEED + omega * L) / (2 * R);
    // float v_right = (2 * SPEED - omega * L) / (2 * R);

    /**
     * @brief Derivative part.
     */
    float error_dot;

    /**
     * @brief Calculate error and transform it from degrees to radians.
     */
    float error = (input_data.desired_heading - input_data.current_heading);
    /**
     * @todo Line below makes sure this will not go over [-pi, pi].
     * This may not be neccessary.
     */
    error = ANGLE_SAFEGUARD(error);

    // error_dot = error - old_error;
    error_dot = ERROR_DOT_ALPHA * ((error - old_error) + old_error_dot);
    error_hat += error;

    float omega = kP * error + (kD / ALGO_DELTA_TIME) * error_dot + (kI * ALGO_DELTA_TIME) * error_hat;

    ESP_LOGI(TAG, "error: %.2f, error_hat: %.2f, error_dot: %.2f", error, error_hat, error_dot);
    /**
     * @todo Line below makes sure this will not go over [-pi, pi].
     * This may not be neccessary.
     */
    // omega = ANGLE_SAFEGUARD(error);
    // omega = error;
    old_error = error;
    old_error_dot = error_dot;

    ESP_LOGI(TAG, "Omega: %.2f", omega * RAD_TO_DEG);

    // omega = 0.0;

    /**
     * @brief PWM calculation will result in values between 0 and 1.
     */
    // motor_control_output_data.pwm1 = (omega + 180.0) / 360.0;
    // motor_control_output_data.pwm1 = -(omega + 180.0) / 360.0;

    motor_control_output_data.pwm1 = motor_calculate_pwm1(omega);
    motor_control_output_data.pwm2 = motor_calculate_pwm2(omega);

    motor_perform_control();
}

/**
 * @brief Calculate PWM duty cycle for the left motor.
 */
static float motor_calculate_pwm1(float omega)
{
    float pwm;
    if (omega <= (-M_PI / 2.0))
    {
        pwm = SPEED_MIN_R;
    }
    else if ((omega > (-M_PI / 2.0) && (omega < 0)))
    {
        pwm = (-(SPEED_MIN_R - SPEED_MID_R) / (M_PI / 2.0)) * omega + SPEED_MID_R;
    }
    else if ((omega >= 0) && (omega < M_PI))
    {
        pwm = ((SPEED_MAX_R - SPEED_MID_R) / M_PI) * omega + SPEED_MID_R;
    }
    else // omega > M_PI
    {
        pwm = SPEED_MAX_R;
    }
    return pwm;
}

/**
 * @brief Calculate PWM duty cycle for the right motor.
 */
static float motor_calculate_pwm2(float omega)
{
    float pwm;
    if (omega <= -M_PI)
    {
        pwm = SPEED_MAX_L;
    }
    else if ((omega > -M_PI) && (omega <= 0))
    {
        pwm = (-(SPEED_MAX_L - SPEED_MID_L) / M_PI) * omega + SPEED_MID_L;
    }
    else if ((omega > 0) && (omega < (M_PI / 2.0)))
    {
        pwm = ((SPEED_MIN_L - SPEED_MID_L) / (M_PI / 2.0)) * omega + SPEED_MID_L;
    }
    else // omega >= (M_PI / 2)
    {
        pwm = SPEED_MIN_L;
    }
    return pwm;
}

void motor_start(float goal_heading)
{
    ESP_ERROR_CHECK(mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0));
    ESP_ERROR_CHECK(mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0));

    /**
     * @brief We initialize old error so we do not have big error in first iteration.
     */
    old_error = goal_heading;
}

void motor_perform_control()
{
    gpio_set_level(DIR1_GPIO_NUM, motor_control_output_data.dir1);
    gpio_set_level(DIR2_GPIO_NUM, motor_control_output_data.dir2);

    ESP_LOGI(TAG, "PWM1 = %.1f, PWM2 = %.1f", motor_control_output_data.pwm1, motor_control_output_data.pwm2);

    ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, motor_control_output_data.pwm1));
    ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_A, motor_control_output_data.pwm2));
}

void motor_reset()
{
    ESP_ERROR_CHECK(mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A));
    ESP_ERROR_CHECK(mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_A));

    motor_control_output_data.pwm1 = 0.0;
    motor_control_output_data.pwm2 = 0.0;

    old_error = 0.0;
    old_error_dot = 0.0;
    error_hat = 0.0;
}