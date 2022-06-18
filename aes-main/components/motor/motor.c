#include "motor.h"
#include "driver/mcpwm.h"

#include <math.h>

// constants
#define SPEED 0.5F
#define PWM_FREQUENCY 50000U

#define FINISH_HEADING 30.0

#define DIR1_GPIO_NUM GPIO_NUM_18
#define DIR2_GPIO_NUM GPIO_NUM_5
#define PWM1_GPIO_NUM GPIO_NUM_17
#define PWM2_GPIO_NUM GPIO_NUM_16

#define ANGLE_SAFEGUARD(x) (atan2f(sinf(x), cosf(x)))
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
 * @todo Fill in the values.
 */
#define kP (0.0)
#define kD (0.0)
#define kI (0.0)

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
static motor_control_input_data_type motor_control_input_data;
static motor_control_output_data_type motor_control_output_data;

/**
 * @brief Variable below should be initialized to desired heading
 * (since we start from heading 0 degrees).
 * This will result in error_dot ≈ 0 in first iteration.
 */
static float old_error = FINISH_HEADING;
static float error_hat = 0;

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

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM1_GPIO_NUM);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PWM2_GPIO_NUM);

    mcpwm_config_t mcpwm_config = {
        .frequency = PWM_FREQUENCY,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER};

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &mcpwm_config);
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
    float error = (input_data.desired_heading - input_data.current_heading) * DEG_TO_RAD;
    /**
     * @todo Line below makes sure this will not go over [-pi, pi].
     * This may not be neccessary.
     */
    error = ANGLE_SAFEGUARD(error);

    error_dot = error - old_error;
    error_hat += error;
    float omega = kP * error + (kD / ALGO_DELTA_TIME) * error_dot + (kI * ALGO_DELTA_TIME) * error_hat;

    /**
     * @todo Line below makes sure this will not go over [-pi, pi].
     * This may not be neccessary.
     */
    omega = ANGLE_SAFEGUARD(error);
    old_error = error;

    /**
     * @brief PWM calculation will result in values between 0 and 1.
     */
    // motor_control_output_data.pwm1 = (omega + 180.0) / 360.0;
    // motor_control_output_data.pwm1 = -(omega + 180.0) / 360.0;

    motor_control_output_data.pwm1 = motor_calculate_pwm1(omega);
    motor_control_output_data.pwm2 = motor_calculate_pwm2(omega);
}

/**
 * @brief Calculate PWM duty cycle for the left motor.
 */
static float motor_calculate_pwm1(float omega)
{
    float pwm;
    if (omega <= (-M_PI / 2))
    {
        pwm = 0;
    }
    else if ((omega > (-M_PI / 2) && (omega < 0)))
    {
        pwm = (2 / M_PI) * omega;
    }
    else // omega >= 0
    {
        pwm = 1;
    }
    return pwm;
}

/**
 * @brief Calculate PWM duty cycle for the right motor.
 */
static float motor_calculate_pwm2(float omega)
{
    float pwm;
    if (omega <= 0)
    {
        pwm = 1;
    }
    else if ((omega > 0) && (omega < (M_PI / 2)))
    {
        pwm = (-2 / M_PI) * omega;
    }
    else // omega >= (M_PI / 2)
    {
        pwm = 0;
    }
    return pwm;
}

void motor_perform_control()
{
    gpio_set_level(DIR1_GPIO_NUM, motor_control_output_data.dir1);
    gpio_set_level(DIR2_GPIO_NUM, motor_control_output_data.dir2);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, motor_control_output_data.pwm1);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, motor_control_output_data.pwm2);
}

void motor_reset()
{
    motor_control_output_data.pwm1 = 0.0;
    motor_control_output_data.pwm2 = 0.0;
    motor_perform_control();

    error_hat = 0.0;
    old_error = FINISH_HEADING;
}

void motor_update_control(float current_heading, float expected_heading)
{
    // drive forward if current_heading == expected_heading
    // rotate clockwise when current_heading > expected_heading
    // rotate counterclockwise when current_heading < expected_heading

    /**
     * @todo Implement interpolation array
     * X: heading difference [deg]
     * Y: turn "strength" [%]
     */

    /**
     * @todo Total heading computation
     * Function of:
     * - angle to obstacle
     * - angle to goal (calculated from current heading and expected heading)
     * Outputs angle towards which we should drive
     */
}