
uint32_t IC_Val1=0;
uint32_t IC_Val2=0;
uint32_t Difference=0;
uint8_t Is_First_Captured=0;
uint8_t Distance= 0;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
uint8_t buffer[64];

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{

		input_capture();
	}
}

//*delay us*//
void delay(uint16_t time){
__HAL_TIM_SetCounter(&htim2,0);
	while(__HAL_TIM_GetCounter (&htim2)<time);

}

//HCSR04_Read*//
void HCSR04_Read (void){

HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}

//HCSR04_Read*//
void input_capture(void)
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1); //(first we catch RISING edge) read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(&htim1, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}
			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			sprintf(buffer, "%d\n", Distance);
			CDC_Transmit_FS(buffer, strlen((buffer == 0)? 1:buffer ));//sending it to SDA

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
			}
}


