#include "motor.h"
#include "driver/mcpwm.h"

// constants
#define SPEED 0.5F
#define PWM_FREQUENCY 50000U

#define DIR1_GPIO_NUM GPIO_NUM_18
#define DIR2_GPIO_NUM GPIO_NUM_5
#define PWM1_GPIO_NUM GPIO_NUM_17
#define PWM2_GPIO_NUM GPIO_NUM_16

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

// local function declarations
static void motor_calculate_control();
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

void motor_calculate_control()
{
    motor_control_output_data.dir1 = 1;
    motor_control_output_data.dir2 = 1;

    if (motor_control_input_data.angle >= 0)
    {
        motor_control_output_data.pwm1 = SPEED;
        motor_control_output_data.pwm2 = SPEED * (cosf(2 * motor_control_input_data.angle) + 1) / 2;
    }
    else
    {
        motor_control_output_data.pwm1 = SPEED * (cosf(2 * motor_control_input_data.angle) + 1) / 2;
        motor_control_output_data.pwm2 = SPEED;
    }
}

void motor_perform_control()
{
    gpio_set_level(DIR1_GPIO_NUM, motor_control_output_data.dir1);
    gpio_set_level(DIR2_GPIO_NUM, motor_control_output_data.dir2);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, motor_control_output_data.pwm1);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, motor_control_output_data.pwm2);
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