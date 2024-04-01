#include "stm32f103_system.h"
#include "stm32f1xx.h" // Include necessary headers for STM32F103

volatile uint32_t tick_count = 0;

volatile uint32_t cpu_load = 0;
uint32_t max_cpu_load = 0;

void SysTick_Handler(void)
{
    tick_count++;
}

/*--------------------------------------------------------------------------*\

 Function:      init_systick()

 Description:   initalizes SysTick timer to 1ms / tick, assumes
                SystemCoreClockUpdate() has been called post RCC config.

 Parameters:    void
 Returns:       void

\*--------------------------------------------------------------------------*/

void init_systick(void)
{
    int tick_time = SystemCoreClock / 1000; // Generate interrupt each 1 ms
    SysTick_Config(tick_time);              // Configure systick timer
}

void busy_tick()
{
    // Perform your application tasks here

    // Sample CPU load every 1000 ms
    if (tick_count >= 1000)
    {
        cpu_load = 100 - ((1000 - (tick_count % 1000)) * 100 / 1000); // Calculate CPU load
        tick_count = 0;                                               // Reset tick count
        // Do something with cpu_load value (e.g., send it over UART)
    }
}

// Function to measure CPU usage
uint32_t measure_cpu_usage()
{
    if (cpu_load > max_cpu_load)
    {
        max_cpu_load = cpu_load; // Update max CPU load
    }
    // Calculate the CPU usage as the ratio of busy time to total time
    return max_cpu_load;
}

uint8_t init_clock()
{
    // Enable HSE (External High-Speed Clock)
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ; // Wait until HSE is ready

    // Configure PLL (PLLCLK = HSE * 9 = 72MHz)
    RCC->CFGR |= RCC_CFGR_PLLSRC;   // Select HSE as PLL source
    RCC->CFGR |= RCC_CFGR_PLLMULL9; // PLL multiplication factor = 9

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ; // Wait until PLL is ready

    // Set Flash latency
    FLASH->ACR |= FLASH_ACR_LATENCY_2;

    // Configure AHB/APBx prescalers to provide the desired system clock frequency
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  // AHB prescaler: SYSCLK not divided
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1 prescaler: HCLK divided by 2
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // APB2 prescaler: HCLK not divided

    // Select PLL as system clock source
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;                    // Wait until PLL is used as the system clock
    SystemCoreClockUpdate(); // calculate the SYSCLOCK value
    // Assuming SysTick timer is used for measuring CPU usage
    // Configure SysTick timer
    return SysTick_Config(SystemCoreClock / 1000); // Configure SysTick to trigger every 1ms
}

void init_rtc()
{
    // Code to initialize the Real-Time Clock (RTC)
    // Example: configuring the RTC with default settings
    // (Assuming RTC peripheral is enabled and configured elsewhere)
    // RTC->CRL &= ~(RTC_CRL_RTOFF); // Disable write protection
    // while (!(RTC->CRL & RTC_CRL_RTOFF)); // Wait for write protection disabled
    // RTC->CRL |= RTC_CRL_CNF; // Enter configuration mode
    // RTC->PRLH = 0x0000; // Set prescaler (MSB)
    // RTC->PRLL = 0x7FFF; // Set prescaler (LSB)
    // RTC->CNTH = 0x0000; // Set counter high (MSB)
    // RTC->CNTL = 0x0000; // Set counter low (LSB)
    // RTC->CRL &= ~(RTC_CRL_CNF); // Exit configuration mode
    // while (!(RTC->CRL & RTC_CRL_RTOFF)); // Wait for configuration mode exited
}

// Function to get the microsecond timestamp

void enableDWT(void)
{
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

uint32_t micros(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}

uint32_t millis(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000);
}
// Other setup functions can be added as needed
