# STM32F103_TB6612_DRIVER Closed Loop



## Description
This project aims to provide a driver for the TB6612 dual motor driver along with encoder support for closed-loop control. Please note that this project is currently a work in progress and is not yet completed. Use it cautiously and feel free to contribute to its development.

## Overview

The TB6612 is a popular dual motor driver capable of driving two DC motors or one stepper motor. This project extends its functionality by integrating encoder support for closed-loop control. The encoder feedback enables precise motor position and speed control, making it suitable for robotics applications

## Features
- Support for two quadrature rotary encoders on PA0 PA1 and PA6 PA7.
- Serial interface for debugging.
- Support for the TB6612 dual motor driver.
- Encoder integration for closed-loop control.
- Precise motor position control.
- esigned for robotics and similar applications.


## Installation
1. Clone or download the repository.
2. Copy the necessary files into your STM32F103C8T6 project.
3. The following tools are used for these projects:
	- ARM-GCC compiler toolchain.
	- st-link flash tool using an ST-LINK V2 USB programmer.
	- Official STM32 CMSIS files as part of their STM32Cube MCU packages.
4. set the path in the makefile

## Usage
 -  Initialize the motor, pid and encoder module by calling the initialization function.

 ## FLASHING
 - run make clean
 - run make swd_flash



## Examples
```in main.c

#define NUMBER_OF_MOTORS 2


encoder_t enc[NUMBER_OF_MOTORS] = {
    {&(TIM3->CNT)},
    {&(TIM2->CNT)}};

/* Initialise PID controller */
PIDController pid = {PID_KP, PID_KI, PID_KD,x
                     PID_TAU,
                     PID_LIM_MIN, PID_LIM_MAX,
                     PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                     SAMPLE_TIME_S};

motor_parameter_t mp[NUMBER_OF_MOTORS] = {
    {MOTOR_DIR_PORT_A, 3, 4, &(TIM4->CCR4), 9, 0},  // id, dir_port, pinA, pinB, pwm_port,pwm_pin, direction inversed
    {MOTOR_DIR_PORT_B, 12, 5, &(TIM4->CCR3), 8, 1}, // id, dir_port, pinA, pinB, pwm_port,pwm_pin, direction inversed
};
```

## Resources
- RM0008 Reference manual page 329
- https://github.com/getoffmyhack/STM32F103-Bare-Metal
- https://www.edwinfairchild.com/2019/04/interface-rotary-encoder-right-way.html


## Credits
https://https://github.com/pms67/PID
https://www.edwinfairchild.com/2019/04/interface-rotary-encoder-right-way.html
https://github.com/getoffmyhack/STM32F103-Bare-Metal

