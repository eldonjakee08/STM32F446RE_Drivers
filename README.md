# STM32F446RE Peripheral Drivers
Custom peripheral drivers for the STM32F446 microcontroller, developed using a bare-metal programming approach without relying on the manufacturer's Hardware Abstraction Layer (HAL).

This is not an attempt to reinvent the wheel nor do i claim that this runs faster or better than the existing manufacturer HAL. 

This instead, is my way of learning the inner workings of an MCU.

# Supported Peripheral Drivers
1. GPIO Drivers 
2. SPI Drivers
3. I2C Drivers
4. USART/UART Drivers


<br>Depending on the project needs in the future, I might add more peripheral drivers. 

# How To Use The Driver
1. Make sure to include the following header files in your main.c file.
   <br><img width="1399" height="330" alt="image" src="https://github.com/user-attachments/assets/c0ac0037-8196-4c69-9f13-b1ae02a491aa" />
2. Once included, you can now call the driver APIs in your application code. Refer to the peripheral driver source files for available APIs.  
