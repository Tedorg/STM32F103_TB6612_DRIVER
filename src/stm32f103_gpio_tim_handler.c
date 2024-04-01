#include "stm32f103_gpio_tim_handler.h"
#include "stm32f1xx.h"
#include <stdio.h>
#include <stdint.h>

static void init_gpio_IO_port(GPIO_TypeDef *PORT);

// Function to initialize GPIO pins
void init_gpio_output(GPIO_TypeDef *PORT, uint16_t PIN)
{

    uint32_t tmpreg = 0x00, pinmask_mode = 0x00, pinmask_cnf = 0x00;


    init_gpio_IO_port(PORT);
    if (PIN < 8)
    {
        tmpreg = PORT->CRL;
        pinmask_mode = 0x3U << ((PIN * 4));
        pinmask_cnf = 0x3U << ((PIN * 4)+(02U));
        tmpreg &= ~((0x3U << pinmask_cnf)|((0x3U<<pinmask_mode)));
        tmpreg |= (0x2U << pinmask_cnf);
        tmpreg |= (0x2U << pinmask_mode);}
    else
    {
        tmpreg = PORT->CRH;
        
    }
        tmpreg = PORT->CRL;
        pinmask_mode = 0x3U << (((PIN-8U) * 4));
        pinmask_cnf = 0x3U << (((PIN-8U) * 4)+(02U));
        tmpreg &= ~((0x3U << pinmask_cnf)|((0x3U<<pinmask_mode)));
        tmpreg |= (0x2U << pinmask_cnf);
        tmpreg |= (0x2U << pinmask_mode);


    // TODO: check which *PORt is passed by refernce for examble GPI
}

 static void init_gpio_IO_port(GPIO_TypeDef *PORT)
{
    if (PORT == GPIOB)
    {
        RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // Enable GPIOB clock
    }
    if (PORT == GPIOA)
    {
        RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Enable GPIOA clock
    }
    if (PORT == GPIOC)
    {
        RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // Enable GPIOC clock
    }
}

// // Function to initialize Timer
// void Timer_Init(void) {
//     // TODO: Add Timer initialization code here
// }

// // Function to handle GPIO interrupt
// void GPIO_IRQHandler(void) {
//     // TODO: Add GPIO interrupt handling code here
// }

// // Function to handle Timer interrupt
// void Timer_IRQHandler(void) {
//     // TODO: Add Timer interrupt handling code here
// }

// int main(void) {
//     // Initialize GPIO pins
//     GPIO_Init();

//     // Initialize Timer
//     Timer_Init();

//     while (1) {
//         // TODO: Add main code here
//     }
// }