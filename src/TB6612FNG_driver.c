#include "TB6612FNG_driver.h"
#include "main.h"
#include <stdlib.h>
#include <stdio.h>
#include "stm32f1xx.h"
#include "encoder.h"
#include "PID.h"
#include "uart.h"

#define MAX_NUM_MOTORS 2

system_state_t sys_st = IDLE;

/*********************** privat functions           *************************/
void update_speed(motor_state_t *st, motor_controller_t *motor);
void update_position(motor_state_t *setpoint, motor_controller_t *motor);
/****************************************************************************/

int number_of_motors = 0;
float out_motors[MAX_NUM_MOTORS];

//    The STBY pin  must be driven high (2.7 V – 5.5 V) in order to enable the driver.
//     - PB03 and PB04 pins control the direction of the motor connected to the OUT1 and OUT2 pins.
//     - PB12 and PB05 pins control the direction of the motor connected to the OUT3 and OUT4 pins.
//     - PB09 PWM controls the speed of the motor connected to the OUT1 and OUT2 pins.
//     - PB08 PWM input controls the speed of the motor connected to the OUT3 and OUT4 pins.
//   */

motor_controller_t controller[MAX_NUM_MOTORS];

// Initialize controller with PID parameters and motor parameters
void motorcontroller_init(encoder_t *enc, PIDController pid, motor_parameter_t *motor, int num_motors)
{
    /* peripherial are set manually, use Abstrationcs in this header-file to set GPIO and PWM   */
    motorcontroller_init_gpio();
    number_of_motors = num_motors;

    // Initialize each controller with PID parameters and motor parameters
    for (int i = 0; i < num_motors; i++)
    {
        controller[i].m = motor[i];
        controller[i].enc = enc[i];
        controller[i].pid = pid;
        init_encoder(&controller[i].enc, motor[i].pwm_pin);
        PIDController_Init(&controller[i].pid);
    }
}

// Update controller with new setpoint and feedback
void update_motor_controller(motor_state_t *state)
{
    for (int i = 0; i < number_of_motors; i++)
    {
        update_encoder(&controller[i].enc);
        switch (sys_st)
        {
        case SPEED_MODE:
            update_speed(&state[i], &controller[i]);
            break;
        case POSITION_MODE:
            update_position(&state[i], &controller[i]);
            break;

        default:
            // Add code here for idle mode
            break;
        }
    }
}

void update_speed(motor_state_t *s, motor_controller_t *m)
{
    double target_speed = s->target_speed;
    s->out_speed = PIDController_Update(&m->pid, target_speed, m->enc.speed) + s->out_speed;
    uint8_t dir = s->out_speed < 0 ? 0 : 1;
    uint32_t pwm = (uint32_t)abs(s->out_speed);
    dir_motor(m, dir);
    pwm_motor(m, pwm);
}

void update_position(motor_state_t *s, motor_controller_t *m)
{
    double target_position = s->target_position;
    s->out_position = PIDController_Update(&m->pid, target_position, (double)m->enc.position);
    uint8_t dir = s->out_position < 0 ? 0 : 1;
    uint32_t pwm = (uint32_t)abs(s->out_position);
    dir_motor(m, dir);
    pwm_motor(m, pwm);
}

void dir_motor(motor_controller_t *c, uint8_t dir)
{
    if (dir)
    {
        GPIOB->ODR |= 1 << c->m.dir_pin_A;    // Set the Pin PB3
        GPIOB->ODR &= ~(1 << c->m.dir_pin_B); // Set the Pin PB3; // Reset the Pin PB3
    }
    else
    {
        GPIOB->ODR &= ~(1 << c->m.dir_pin_A); // Set the Pin PB3
        GPIOB->ODR |= 1 << c->m.dir_pin_B;    // Set the Pin PB3; // Reset the Pin PB3
    }
}

void pwm_motor(motor_controller_t *c, uint32_t pwm)
{
    *(c->m.ccr) = pwm;
}

/* mainly used for PID tuning  */

void print_motor_controller(uint8_t motor_id, motor_state_t *s)
{
    if (sys_st == SPEED_MODE)
    {
        switch (motor_id)
        {
        case 0:
            printf("%4.3f %4.3f %d %d", s[motor_id].target_speed, controller[motor_id].enc.speed, 0, 250); // 0 and 200 are onkly used to prevent arduino ^plotter autoscaling
            break;
        case 1:
            printf("%4.3f %4.3f %d %d", s[motor_id].target_speed, controller[motor_id].enc.speed, 0, 250); // 0 and 200 are onkly used to prevent arduino ^plotter autoscaling
            break;
        default:
            for (int i = 0; i < motor_id; i++)
            {
                printf("%4.3f %4.3f %d %d", s[i].target_speed, controller[i].enc.speed, 0, 250); // 0 and 200 are onkly used to prevent arduino ^plotter autoscaling
            }
            break;
        }
        printf("\n");
    }
    else if (sys_st == POSITION_MODE)
    {
        switch (motor_id)
        {
        case 0:
            printf("%4.3f %ld %d %d", s[motor_id].target_position, controller[motor_id].enc.position, -1300, 1300); // 0 and 200 are onkly used to prevent arduino ^plotter autoscaling
            break;
        case 1:
            printf("%4.3f %ld %d %d", s[motor_id].target_position, controller[motor_id].enc.position, -1300, 1300); // 0 and 200 are onkly used to prevent arduino ^plotter autoscaling
            break;
        default:
            for (int i = 0; i < motor_id; i++)
            {
                printf("%4.3f %ld %d %d", s[i].target_position, controller[i].enc.position, -1300, 130); // 0 and 200 are onkly used to prevent arduino ^plotter autoscaling
            }
            break;
        }
        printf("\n");
    }
}

uint8_t set_mode_controller(uint8_t mode)
{
    switch (mode)
    {
    case 0:
        sys_st = IDLE;
        break;
    case 1:
        sys_st = SPEED_MODE;
        break;
    case 2:
        sys_st = POSITION_MODE;
        break;
    default:
        return 1;
    }

    return 0;
}

uint8_t set_controller(char *data, motor_state_t *s, uint8_t lenght)
{
    if (lenght < 3)
        return 1;
    double d = atof(&data[0]);
    switch (sys_st)
    {
    case SPEED_MODE:
        s->target_speed = d;
        break;
    case POSITION_MODE:
        s->target_position = d;
        break;
    }
    return 0;
}

uint8_t set_gains_motor_controller(char *data, uint8_t lenght)
{
    if (lenght < 3)
        return 1;
    if (data[0] == 'p' || data[0] == 'd' || data[0] == 'i')
    {
        float gain = 0.0f;
        gain = atof(&data[1]);
        for (int i = 0; i < number_of_motors; i++)
        {
            switch (data[0])
            {
            case 'p':
                controller[i].pid.Kp = gain;
                printf("Motor id: %d  Kp: %c  Value: %f\n", i, data[0], controller[i].pid.Kp);
                break;
            case 'd':
                controller[i].pid.Kd = gain;
                printf("Motor id: %d  Kd: %c  Value: %f\n", i, data[0], controller[i].pid.Kd);
                break;
            case 'i':
                controller[i].pid.Ki = gain;
                printf("Motor id: %d  Ki: %c  Value: %f\n", i, data[0], controller[i].pid.Ki);
                break;
            default:
                break;
            }
        }
    }
    else
    {
        return 1;
    }
    return 0;
}

uint8_t motorcontroller_init_gpio(void)
{
    /* init gpio motor
        init pwm
        DIsable SWJ Interface
    */
    // DIsable SWJ Interface to use PB3+4 Ref.Manual s.184
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;      // Enable A.F. clock
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_DISABLE; // JTAG is disabled, SWD is enabled
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;      // Enable GPIOB clock
    /* PB03 and PB04 pins control the direction of the motor connected to the OUT1 and OUT2 pins. (links)
    left motor
    */
    MOTOR_DIR_PORT_A->MOTOR_DIR_CR_A_1 &= ~(MOTOR_DIR_RESET_BITS_A_1); // Clear pin configuration
    MOTOR_DIR_PORT_A->MOTOR_DIR_CR_A_2 &= ~(MOTOR_DIR_RESET_BITS_A_2); // Clear pin configuration
    MOTOR_DIR_PORT_A->MOTOR_DIR_CR_A_1 |= MOTOR_DIR_SET_BITS_A_1;      // Set output mode to 50MHz
    MOTOR_DIR_PORT_A->MOTOR_DIR_CR_A_2 |= MOTOR_DIR_SET_BITS_A_2;      // Set output mode to 50MHz

    MOTOR_DIR_PORT_B->MOTOR_DIR_CR_B_1 &= ~(MOTOR_DIR_RESET_BITS_B_1); // Clear pin configuration
    MOTOR_DIR_PORT_B->MOTOR_DIR_CR_B_2 &= ~(MOTOR_DIR_RESET_BITS_B_2); // Clear pin configuration
    MOTOR_DIR_PORT_B->MOTOR_DIR_CR_B_1 |= MOTOR_DIR_SET_BITS_B_1;      // Set output mode to 50MHz
    MOTOR_DIR_PORT_B->MOTOR_DIR_CR_B_2 |= MOTOR_DIR_SET_BITS_B_2;      // Set output mode to 50MHz

    /* PWM controls the speed of the motor. */

    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // Enable TIM4 clock

    MOTOR_PWM_PORT_A->MOTOR_PWM_CR_A &= ~(MOTOR_PWM_RESET_A); // Clear pin configuration
    MOTOR_PWM_PORT_A->MOTOR_PWM_CR_A |= MOTOR_PWM_SET_A;      // Set output mode to 50MHz

    MOTOR_PWM_PORT_B->MOTOR_PWM_CR_B &= ~(MOTOR_PWM_RESET_B); // Clear pin configuration
    MOTOR_PWM_PORT_B->MOTOR_PWM_CR_B |= MOTOR_PWM_SET_B;      // Set output mode to 50MHz

    // set prescale to be F = 1 MHz ; T = 0.000001 = 1µs
    // Fcounter = Fprescale/(PSC + 1) -> PSC = (Fprescale / Fcounter) - 1
    uint16_t prescale = (SystemCoreClock / PRESCALE_1MHZ);

    MOTOR_PWM_TIM->PSC = prescale - 1;
    // set period to be 50Hz / 20ms
    uint16_t period = (SystemCoreClock / prescale / PULSE_FREQ);
    MOTOR_PWM_TIM->ARR = period;

    MOTOR_PWM_TIM->MOTOR_PWM_TIM_CH_A = PULSE_MIN; //
    MOTOR_PWM_TIM->MOTOR_PWM_TIM_CH_B = PULSE_MIN; //

    // // configure output compare
    MOTOR_PWM_TIM->MOTOR_PWM_TIM_CCMR_A |= MOTOR_PWM_TIM_CCMR_MODE_A;
    MOTOR_PWM_TIM->MOTOR_PWM_TIM_CCMR_B |= MOTOR_PWM_TIM_CCMR_MODE_B;

    MOTOR_PWM_TIM->MOTOR_PWM_TIM_CCER_A |= MOTOR_PWM_TIM_CCER_MODE_A;
    MOTOR_PWM_TIM->MOTOR_PWM_TIM_CCER_B |= MOTOR_PWM_TIM_CCER_MODE_B;

    MOTOR_PWM_TIM->MOTOR_PWM_TIM_CR_A |= MOTOR_PWM_TIM_CR_EN_A;

    return 0;
}
