#include "motor.h"

#include "algo.h"
#include "calc_utils.h"

#include "driver/mcpwm.h"

#include <math.h>
#include <float.h>

// constants
/**
 * @brief Minimum and maximum vehicle speed, in percent of maximum motors speed.
 * Values from 0 to 100.
 * Minimum vehicle speed should be set as lowest value which do not make motors
 * turn (because of motor/tracks/ground friction).
 */
#define SPEED_MIN (7.0F)
#define SPEED_MID (25.0F)
#define SPEED_MAX (28.0F)

/**
 * @brief Motor correction coefficients
 * L/R - left/right
 * F/B - forwards/backwards
 */
#define MOTOR_CORRECTION_L_F (1.0F)
#define MOTOR_CORRECTION_R_F (0.885F)

#define MOTOR_CORRECTION_L_B (0.885F)
#define MOTOR_CORRECTION_R_B (1.0F)

/**
 * @brief Forwards speed constants.
 */
#define SPEED_MIN_L_F (SPEED_MIN * MOTOR_CORRECTION_L_F)
#define SPEED_MIN_R_F (SPEED_MIN * MOTOR_CORRECTION_R_F)

#define SPEED_MID_L_F (SPEED_MID * MOTOR_CORRECTION_L_F)
#define SPEED_MID_R_F (SPEED_MID * MOTOR_CORRECTION_R_F)

#define SPEED_MAX_L_F (SPEED_MAX * MOTOR_CORRECTION_L_F)
#define SPEED_MAX_R_F (SPEED_MAX * MOTOR_CORRECTION_R_F)

/**
 * @brief Backwards speed constants.
 */
#define SPEED_MIN_L_B (SPEED_MIN * MOTOR_CORRECTION_L_B)
#define SPEED_MIN_R_B (SPEED_MIN * MOTOR_CORRECTION_R_B)

#define SPEED_MID_L_B (SPEED_MID * MOTOR_CORRECTION_L_B)
#define SPEED_MID_R_B (SPEED_MID * MOTOR_CORRECTION_R_B)

#define SPEED_MAX_L_B (SPEED_MAX * MOTOR_CORRECTION_L_B)
#define SPEED_MAX_R_B (SPEED_MAX * MOTOR_CORRECTION_R_B)

/**
 * @brief Forward switch threshold.
 * If requested angle goes over the threshold,
 * the vehicle will start moving one of the tracks
 * in the reverse direction.
 */
#define FST (M_PI / 4.0)

/**
 * @brief Turn speedup threshold.
 * Creates an interpolation point of angle which
 * will result in maximum speed of both motors in reverse direction.
 */
#define TST (M_PI / 2.0)

/**
 * @brief Threshold below which integral part
 * of the PID regulator is used.
 */
#define INTEGRAL_ACTIVATION_THRESHOLD (FST)

/**
 * @brief Maximum/minimum value of the integral term.
 */
#define INTEGRAL_BOUND (M_PI)

/**
 * @brief PWM1 (right motor) linear function coefficients.
 */
#define PWM1_A1 ((SPEED_MAX_R_B - SPEED_MIN_R_B) / (FST - TST))
#define PWM1_B1 (PWM1_A1 * FST + SPEED_MIN_R_B)

#define PWM1_A2 ((SPEED_MID_R_F - SPEED_MIN_R_F) / (FST))
#define PWM1_B2 (SPEED_MID_R_F)

#define PWM1_A3 ((SPEED_MAX_R_F - SPEED_MID_R_F) / (TST))
#define PWM1_B3 (SPEED_MID_R_F)

/**
 * @brief PWM2 (left motor) linear function coefficients.
 */
#define PWM2_A1 ((SPEED_MID_L_F - SPEED_MAX_L_F) / (TST))
#define PWM2_B1 (SPEED_MID_L_F)

#define PWM2_A2 ((SPEED_MIN_L_F - SPEED_MID_L_F) / (FST))
#define PWM2_B2 (SPEED_MID_L_F)

#define PWM2_A3 ((SPEED_MIN_L_B - SPEED_MAX_L_B) / (FST - TST))
#define PWM2_B3 (PWM2_A3 * (-FST) + SPEED_MIN_L_B)

/**
 * @todo Maybe correct speeds on motors each.
 * Vehicle is driving a little bit to the left -
 * correct that by giving left motor a little more speed.
 */

#define PWM_FREQUENCY 5000U

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)

// /**
//  * @brief This needs to refactored, because we probably will not
//  * use finish heading, but end point.
//  */
// #define FINISH_HEADING (30.0 * DEG_TO_RAD)

#define DIR1_GPIO_NUM GPIO_NUM_18
#define DIR2_GPIO_NUM GPIO_NUM_5
#define PWM1_GPIO_NUM GPIO_NUM_17
#define PWM2_GPIO_NUM GPIO_NUM_16

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
#define kP (1.5)
#define kD (0.2)
#define kI (1.0)

/**
 * @brief Derivative term implemented as IIR high-pass filter.
 * Alpha value adjustable below.
 */
#define ERROR_DOT_ALPHA (0.7)

// structs

// global variables
motor_control_output_data_type motor_control_output_data;

// local variables
static const char *TAG = "motor";

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
static int motor_calculate_dir1(float omega);
static int motor_calculate_dir2(float omega);
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

void compare_float(float received, float expected)
{
    float epsilon = 1.0 / 16384;
    ESP_LOGI(TAG, "Comparing received %f to expected %f", received, expected);
    bool result = fabs(received - expected) < epsilon;
    if (result)
    {
        ESP_LOGI(TAG, "Pass");
    }
    else
    {
        ESP_LOGE(TAG, "Fail");
        // abort();
    }
}

void motor_run_tc()
{
    compare_float(motor_calculate_pwm1(DEG_TO_RAD * -181), SPEED_MAX_R_B);
    compare_float(motor_calculate_pwm1(DEG_TO_RAD * -180), SPEED_MAX_R_B);
    compare_float(motor_calculate_pwm1(DEG_TO_RAD * -60), SPEED_MIN_R_B);
    compare_float(motor_calculate_pwm1(DEG_TO_RAD * 0), SPEED_MID_R_F);
    compare_float(motor_calculate_pwm1(DEG_TO_RAD * 180), SPEED_MAX_R_F);
    compare_float(motor_calculate_pwm1(DEG_TO_RAD * 181), SPEED_MAX_R_F);

    compare_float(motor_calculate_pwm2(DEG_TO_RAD * -181), SPEED_MAX_L_F);
    compare_float(motor_calculate_pwm2(DEG_TO_RAD * -180), SPEED_MAX_L_F);
    compare_float(motor_calculate_pwm2(DEG_TO_RAD * 0), SPEED_MID_L_F);
    compare_float(motor_calculate_pwm2(DEG_TO_RAD * 60), SPEED_MIN_L_B);
    compare_float(motor_calculate_pwm2(DEG_TO_RAD * 180), SPEED_MAX_L_B);
    compare_float(motor_calculate_pwm2(DEG_TO_RAD * 181), SPEED_MAX_L_B);
}

void motor_tick(motor_control_input_data_type input_data)
{
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

    float p = kP * error;
    float d = (kD / ALGO_DELTA_TIME) * error_dot;
    float omega = p + d;

    /**
     * @brief Use integral part only if the error is small.
     */
    if (fabsf(error) < INTEGRAL_ACTIVATION_THRESHOLD)
    {
        error_hat += error;
        /**
         * @brief Clamp the integral term.
         */
        if (fabs(error_hat) > INTEGRAL_BOUND)
        {
            error_hat = copysignf(INTEGRAL_BOUND, error_hat);
        }

        float i = (kI * ALGO_DELTA_TIME) * error_hat;
        omega += i;
    }
    else
    {
        error_hat = 0;
    }

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
    motor_control_output_data.dir1 = motor_calculate_dir1(omega);
    motor_control_output_data.dir2 = motor_calculate_dir2(omega);

    // motor_control_output_data.pwm1 = SPEED_MID_R_B;
    // motor_control_output_data.pwm2 = SPEED_MID_L_B;
    // motor_control_output_data.dir1 = 1;
    // motor_control_output_data.dir2 = 1;

    motor_perform_control();
}

/**
 * @brief Calculate PWM duty cycle for the right motor.
 */
static float motor_calculate_pwm1(float omega)
{
    float pwm;
    if (omega <= (-TST))
    {
        pwm = SPEED_MAX_R_B;
    }
    else if ((omega > (-TST)) && (omega < (-FST)))
    {
        pwm = PWM1_A1 * omega + PWM1_B1;
    }
    else if ((omega >= (-FST)) && (omega < 0))
    {
        pwm = PWM1_A2 * omega + PWM1_B2;
    }
    else if ((omega >= 0) && (omega < TST))
    {
        pwm = PWM1_A3 * omega + PWM1_B3;
    }
    else // omega >= TST
    {
        pwm = SPEED_MAX_R_F;
    }
    return pwm;
}

/**
 * @brief Calculate PWM duty cycle for the left motor.
 */
static float motor_calculate_pwm2(float omega)
{
    float pwm;
    if (omega <= (-TST))
    {
        pwm = SPEED_MAX_L_F;
    }
    else if ((omega > (-TST)) && (omega < 0))
    {
        pwm = PWM2_A1 * omega + PWM2_B1;
    }
    else if ((omega >= 0) && (omega < FST))
    {
        pwm = PWM2_A2 * omega + PWM2_B2;
    }
    else if ((omega >= FST) && (omega < TST))
    {
        pwm = PWM2_A3 * omega + PWM2_B3;
    }
    else // omega >= TST
    {
        pwm = SPEED_MAX_L_B;
    }
    return pwm;
}

static int motor_calculate_dir1(float omega)
{
    int dir;
    if (omega <= (-FST))
    {
        dir = 1;
    }
    else
    {
        dir = 0;
    }
    return dir;
}

static int motor_calculate_dir2(float omega)
{
    int dir;
    if (omega > FST)
    {
        dir = 1;
    }
    else
    {
        dir = 0;
    }
    return dir;
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

bool motor_is_turning()
{
    return motor_control_output_data.dir1 || motor_control_output_data.dir2;
}