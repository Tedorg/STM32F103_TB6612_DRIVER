#include "main.h"
#include "stm32f1xx.h"
#include <stdio.h>
#include <stdint.h>
#include "stm32f103_system.h"
#include "uart.h"
#include "encoder.h"
#include "TB6612FNG_driver.h"

/**********************defines *************************/

#define bufferLength 64          // serial buffer length
char serialBuffer[bufferLength]; // serial buffer

// create simple define abstractions for heartbeat LED
#define LED_PORT GPIOC
#define LED_CR CRH
#define LED_SET GPIO_BSRR_BS13
#define LED_RESET GPIO_BSRR_BR13
#define LED_PORT_RESET_BITS GPIO_CRH_MODE13 | GPIO_CRH_CNF13
#define LED_PORT_SET_BITS GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0
#define LED_CLOCK RCC_APB2ENR_IOPCEN

/****************************************************************************\
 File:          main.c
 Date:
 Description:
 Known bugs/missing features:
 initi gpio and pwm without hardcoding
\****************************************************************************/
/***********************  Privat Functions  *************************/
void led_heartbeat(void);
void system_init(void);
void setup_timer(void);
static void print_debug_info(void);
void serial_get(void);
void parse_message(char *message, uint16_t length);
/*********************** global variables  *************************/
uint32_t time,
    initialTime;

/***********************  DEBUG  *************************/
uint8_t print_state = 4;
/****************************************************************************/

motor_state_t ms[NUMBER_OF_MOTORS] =
    { // Initialize position, speed, and state
        {0},
        {0}};

/* Initialise Encoder controller */
encoder_t enc[NUMBER_OF_MOTORS] = {
    {&(TIM3->CNT)},
    {&(TIM2->CNT)}};

/* Initialise PID controller */
PIDController pid = {PID_KP, PID_KI, PID_KD,
                     PID_TAU,
                     PID_LIM_MIN, PID_LIM_MAX,
                     PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                     SAMPLE_TIME_S};

motor_parameter_t mp[NUMBER_OF_MOTORS] = {
    {MOTOR_DIR_PORT_A, 3, 4, &(TIM4->CCR4), 9, 0},  // id, dir_port, pinA, pinB, pwm_port,pwm_pin, direction inversed
    {MOTOR_DIR_PORT_B, 12, 5, &(TIM4->CCR3), 8, 1}, // id, dir_port, pinA, pinB, pwm_port,pwm_pin, direction inversed
};

void init_led(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;                    // enable GPIO clock for LED
    GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);     // reset pin MODE / CNF
    GPIOC->CRH |= (GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0); // MODE: 50Mhz ouput CNF: PP
}

void led_heartbeat(void)
{
    static uint8_t led_state = 0;

    switch (led_state)
    {
    case 0:
        GPIOC->BSRR = GPIO_BSRR_BS13;
        break;

    case 1:
        GPIOC->BSRR = GPIO_BSRR_BR13;
        break;

    default:
        break;
    }
    led_state = !led_state;
}

void system_init()
{
    init_clock();
    init_systick();
    init_led();
    enableDWT();
    init_uart(USART_BAUD);
}

int main()
{

    system_init();
    motorcontroller_init(enc, pid, mp, 2);
    setup_timer();

    while (1)
    {
        serial_get();
        time = micros();
        if (time - initialTime >= 50000)
        {
            led_heartbeat();
            if (print_state == MOTOR_LEFT)
                print_motor_controller(0, ms);
            if (print_state == MOTOR_RIGHT)
                print_motor_controller(1, ms);
            if (print_state == 2)
                print_motor_controller(2, ms);
            if (print_state == 3)
                print_debug_info();

            initialTime = time;
        }
    }
    return 0;
}

void setup_timer()
{
    uint32_t prescaler_value = 1000;    // Prescaler value
    uint32_t period_value = 720;        // Period value
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // Enable TIM2 Periph clock
    TIM1->PSC = prescaler_value;
    TIM1->ARR = period_value;
    TIM1->DIER |= TIM_DIER_UIE;        /* interrupt on update */
    NVIC_SetPriority(TIM1_UP_IRQn, 3); // Set interrupt priority (0 = highest)
    NVIC_EnableIRQ(TIM1_UP_IRQn);      // Enable the interrupt in the NVIC
    TIM1->CR1 &= ~TIM_CR1_CEN;         /* disable */
    TIM1->CR1 |= TIM_CR1_CEN;          /* enable */
}

void TIM1_UP_IRQHandler(void)
{

    if (TIM1->SR & TIM_SR_UIF)
    {
        update_motor_controller(ms);
        TIM1->SR &= ~TIM_SR_UIF; // Clear the interrupt flag
    }
}

static void print_debug_info()
{
    printf("\n"); // 0 and 200 are onkly used to prevent arduino ^plotter autoscaling
}

void serial_get(void)
{
    // Read input from the serial interface

    if (rxAvailable())
    {
        uint16_t length = egets(serialBuffer, bufferLength);
        parse_message(serialBuffer, length);
    }

    // parse_message(serialBuffer, 64);
}
// Parse the input and execute the command
void parse_message(char *message, uint16_t length)
{
    // Your code to parse and execute the command goes here
    // Example code:
    int parameter = 0;
    switch (message[0])
    {

    case 'M':
        // Switch Mode to Position or Speed
        parameter = atoi(&message[1]);
        set_mode_controller(parameter);
        break;
        // Set target- speed or position
    case 'L':
        if (message[1] != '\0')
            set_controller(&message[1], &ms[MOTOR_LEFT], length);
        break;
    case 'R':
        if (message[1] != '\0')
            set_controller(&message[1], &ms[MOTOR_RIGHT], length);
        break;
        // set pid gains
    case 'd':
        if (message[1] != '\0')
            set_gains_motor_controller(&message[1], length);

        break;
        // Set print mode, this is mainly used for debugging
    case 'p':
        parameter = atoi(&message[1]);
        print_state = parameter;

        break;

    default:
        // Handle invalid command
        break;
    }
}
