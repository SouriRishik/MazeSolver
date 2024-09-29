#ifndef HCSR04_H
#define HCSR04_H

#include "stm32f4xx_hal.h"  // Include the appropriate HAL header file for your STM32 series

// Define the number of sensors


// External TIM handles
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

// Function prototypes
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HCSR04_Trigger();
void Trigger_All_Sensors(void);
void delay(uint16_t time);
void HCSR04_Read(void);
void input_capture(uint8_t sensor_id);
void delay_us (uint16_t us);

// Variables to store sensor data


// Define GPIO ports and pins for each sensor (adjust according to your setup)


#endif // HCSR04_H
