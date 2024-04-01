/*Module encoder should return the speed of the robot in rotation per minute
The encoder module should be able to calculate the speed of the robot based on the number of encoder steps and the time passed
this module should be initialized with the init_encoder function and updated with the update_speed function

use it like this:

in main.c
encoder_t left, right;
init_encoder(&left, &right);
while(1) {
    update_speed(&left, &right);
    print_debug_info(&left, &right);
}
*/

#include "encoder.h"
#include <stdint.h>
#include <stdio.h>
#include "stm32f1xx.h"
#include "stm32f103_system.h"

#define ENCODER_COUNT 1300 // number of encoder steps per wheel revolution
#define MAX_ENCODERS 2     // maximum amount of encoders

/*reset encoder count is halt of max uint16. this helps to prevent faulty vlaue on overflow*/
const int reset_encoder_count = 65535 / 2;
uint8_t used_encoders = 0;

int32_t current_millis = 0, previous_millis = 0;

static float calculate_speed(int count, int interval);
static void init_decoder_TIM2(void);
static void init_decoder_TIM3(void);

uint8_t init_encoder(encoder_t * enc, uint8_t number_of_encoder)
{
    used_encoders = number_of_encoder+1;


    if (number_of_encoder+1 > MAX_ENCODERS)
        return 1;

    if (number_of_encoder == 0)
    {
        init_decoder_TIM2();
        enc->timer = (volatile uint32_t *) TIM2->CNT;
    }
    else 
    {
        init_decoder_TIM3();
        enc->timer = (volatile uint32_t *) TIM3->CNT;
    }
   enc->current_count = 0;
    enc->position = 0;
    enc->speed= 0; // units per minute 
     printf("used encoders %d \n", used_encoders);
    return 0;
}

static float calculate_speed(int count, int interval)
{
    // Calculate speed in units per minute
    //
    float speed = (count / (float)ENCODER_COUNT) * (60000.0 / interval);
    return speed;
}

// ...

void update_encoder(encoder_t *enc)
{
    /*
    Update general values for both encoders
    interval is the time passed since the last update. its measured in milliseconds

    */

    int interval, count_difference;
    current_millis = millis();
    interval = current_millis - previous_millis;

        enc->current_count = enc->timer;
        count_difference = enc->current_count - reset_encoder_count;
    /*poston sums driven distance */
        enc->position += (count_difference);
        enc->speed = calculate_speed(count_difference, interval);
    /*reset encoder count to prevent overflow*/
        enc->timer = reset_encoder_count;

    
 

    previous_millis = current_millis;
}

float get_speed_enc(encoder_t *enc)
{
    return enc->speed;
}

int32_t get_position_enc(encoder_t *enc)
{
    return enc->position;
}

void reset_position_enc(encoder_t *enc)
{
    enc->position = 0;
}

void print_info_enc(encoder_t *enc)
{
    // Print debug information
     printf("Left: %ld, speed: %0.3f Pos: %ld \t Right: %ld, speed: %0.3f Pos: %ld  \n", left->current_count, left->speed, left->position, right->current_count, right->speed, right->position);
}
// ...

// float get_speed(struct encoder* enc) {
//     return enc->speed;
// }

static void init_decoder_TIM3(void)
{

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    // Configure Pins 0 and 1 of Port A as alternate function inputs
    GPIOA->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_CNF7);
    GPIOA->CRL |= (GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1);
    TIM3->ARR = 0xFFFF;
    // Configure encoder interface
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;                // Enable clock for TIM2                                               // TIM3->CR1 = 0;                                     // Disable timer
    TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0; // Capture on TI1 and TI2
    TIM3->SMCR = TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;      // Enable encoder mode
    TIM3->CNT = reset_encoder_count;                   // Reset counter
    TIM3->CR1 |= TIM_CR1_CEN;                          // Enable timer
}

static void init_decoder_TIM2(void)
{
    // Enable clock for GPIO Port A
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    // Configure Pins 0 and 1 of Port A as alternate function inputs
    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_CNF1);
    GPIOA->CRL |= (GPIO_CRL_CNF0_1 | GPIO_CRL_CNF1_1);
    TIM2->ARR = 0xFFFF;
    // Configure encoder interface
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;                // Enable clock for TIM2                                               // TIM2->CR1 = 0;                                     // Disable timer
    TIM2->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0; // Capture on TI1 and TI2
    TIM2->SMCR = TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;      // Enable encoder mode
    TIM2->CNT = reset_encoder_count;                   // Reset counter
    TIM2->CR1 |= TIM_CR1_CEN;                          // Enable timer
}
