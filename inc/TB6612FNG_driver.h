#ifndef TB6612FNG_DRIVER_H
#define TB6612FNG_DRIVER_H
#include <stdint.h>
#include "PID.h"
#include "encoder.h"
#include "stm32f1xx.h"

// MOTOR IDs
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

/* PID Controller parameters */
#define PID_KP 4.0f
#define PID_KI 0.1f
#define PID_KD 0.2f

#define PID_TAU 0.02f

#define PID_LIM_MIN -1000.0f
#define PID_LIM_MAX 1000.0f

#define PID_LIM_MIN_INT -5000.0f
#define PID_LIM_MAX_INT 500.0f

#define SAMPLE_TIME_S 0.01f

// PWM defines
#define PULSE_MIN 0
#define PULSE_FREQ 1000       // 1000 µs = 1 ms
#define PRESCALE_1MHZ 1000000 // 1 Mhz prescale

// Abstractions for Port A OUT 1,2 Direction Pins MOTOR LINKS
#define MOTOR_DIR_PORT_A GPIOB

#define MOTOR_DIR_CR_A_1 CRL
#define MOTOR_DIR_CR_A_2 CRL

#define MOTOR_DIR_SET_A_1 GPIO_BSRR_BS3
#define MOTOR_DIR_SET_A_2 GPIO_BSRR_BS4

#define MOTOR_DIR_RESET_A_1 GPIO_BSRR_BR3
#define MOTOR_DIR_RESET_A_2 GPIO_BSRR_BR4

#define MOTOR_DIR_RESET_BITS_A_1 GPIO_CRL_MODE3 | GPIO_CRL_CNF3
#define MOTOR_DIR_RESET_BITS_A_2 GPIO_CRL_MODE4 | GPIO_CRL_CNF4

#define MOTOR_DIR_SET_BITS_A_1 GPIO_CRL_MODE3_0 //  MODE: 10Mhz Push-Pull
#define MOTOR_DIR_SET_BITS_A_2 GPIO_CRL_MODE4_0 // MODE: 10Mhz Push-Pull

// Abstractions for Port B OUT 1,2 Direction Pins MOTOR RECHTS
#define MOTOR_DIR_PORT_B GPIOB
#define MOTOR_DIR_CR_B_1 CRH
#define MOTOR_DIR_CR_B_2 CRL

#define MOTOR_DIR_SET_B_1 GPIO_BSRR_BS12
#define MOTOR_DIR_SET_B_2 GPIO_BSRR_BS5

#define MOTOR_DIR_RESET_B_1 GPIO_BSRR_BR12
#define MOTOR_DIR_RESET_B_2 GPIO_BSRR_BR5

#define MOTOR_DIR_RESET_BITS_B_1 GPIO_CRH_MODE12 | GPIO_CRH_CNF12
#define MOTOR_DIR_RESET_BITS_B_2 GPIO_CRL_MODE5 | GPIO_CRL_CNF5

#define MOTOR_DIR_SET_BITS_B_1 GPIO_CRH_MODE12_0 // MODE: 10Mhz ouput CNF: PP
#define MOTOR_DIR_SET_BITS_B_2 GPIO_CRL_MODE5_0  // MODE: 10Mhz ouput CNF: PP

// Abstractions for Port A PWM
#define MOTOR_PWM_PORT_TIM RCC_APB1ENR_TIM4EN // Enable TIM4 clock
#define MOTOR_PWM_TIM TIM4
#define MOTOR_PWM_TIM_CH_A CCR4                                         // TIM4 Channel 3
#define MOTOR_PWM_TIM_CCMR_A CCMR2                                      // capture/compare mode regigster 2 RM0008 page 413
#define MOTOR_PWM_TIM_CCMR_MODE_A (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1) // capture/compare mode
#define MOTOR_PWM_TIM_CCER_A CCER                                       // capture/compare enable register
#define MOTOR_PWM_TIM_CCER_MODE_A TIM_CCER_CC4E                         // capture/compare outpu 3
#define MOTOR_PWM_TIM_CR_A CR1                                          // control register
#define MOTOR_PWM_TIM_CR_EN_A TIM_CR1_CEN                               // Bit 0 CEN: Counter enable/disable

#define MOTOR_PWM_PORT_A GPIOB
#define MOTOR_PWM_CR_A CRH
#define MOTOR_PWM_RESET_A (GPIO_CRH_CNF9 | GPIO_CRH_MODE9)                      // Clear pin configuration
#define MOTOR_PWM_SET_A (GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0 | GPIO_CRH_CNF9_1) // Set output mode to 50MHz

// Abstractions for Port B PWM
#define MOTOR_PWM_TIM_B TIM4
#define MOTOR_PWM_TIM_CH_B CCR3                                         // TIM4 Channel 4
#define MOTOR_PWM_TIM_CCMR_B CCMR2                                      // capture/compare mode regigster 2 RM0008 page 413
#define MOTOR_PWM_TIM_CCMR_MODE_B (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1) // capture/compare mode
#define MOTOR_PWM_TIM_CCER_B CCER                                       // capture/compare enable register
#define MOTOR_PWM_TIM_CCER_MODE_B TIM_CCER_CC3E                         // capture/compare outpu 4
#define MOTOR_PWM_TIM_CR_B CR1                                          // control register
#define MOTOR_PWM_TIM_CR_EN_B TIM_CR1_CEN                               // Bit 0 CEN: Counter enable/disable

#define MOTOR_PWM_PORT_B GPIOB
#define MOTOR_PWM_CR_B CRH
#define MOTOR_PWM_RESET_B (GPIO_CRH_CNF8 | GPIO_CRH_MODE8)                      // Clear pin configuration
#define MOTOR_PWM_SET_B (GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0 | GPIO_CRH_CNF8_1) // Set output mode to 50MHz

typedef enum
{
    IDLE,
    SPEED_MODE,
    POSITION_MODE,
} system_state_t;

typedef struct
{
    double out_position;
    double target_position;
    double out_speed;
    double target_speed;
} motor_state_t;

typedef struct
{
    GPIO_TypeDef *dir_port;           // direction pin port
    uint16_t dir_pin_A;               // direction pin  number
    uint16_t dir_pin_B;               // direction pin  number
    volatile uint32_t *ccr;           // direction pin port
    uint16_t pwm_pin;                 // pwm pin number
    uint8_t motor_direction_inversed; // 0 for normal, 1 for inversed
} motor_parameter_t;

typedef struct
{
    encoder_t enc;
    PIDController pid;   // PID controller instance
    motor_parameter_t m; // Motor parameters
} motor_controller_t;

uint8_t motorcontroller_init_gpio(void);
void motorcontroller_init(encoder_t *enc, PIDController pid, motor_parameter_t *motor, int num_motors);
void update_motor_controller(motor_state_t *state);
void dir_motor(motor_controller_t *controller, uint8_t dir);
void pwm_motor(motor_controller_t *controller, uint32_t pwm);
void print_motor_controller(uint8_t motor_id, motor_state_t *s);
uint8_t motorcontroller_init_gpio(void);
uint8_t set_mode_controller(uint8_t mode);
uint8_t set_controller(char *data, motor_state_t *s, uint8_t lenght);
uint8_t set_gains_motor_controller(char *data, uint8_t lenght);
// Declare controllerData as an external variable
extern motor_controller_t motor_controller_data[];

#endif // MOTOR_CONTROLLER_H
