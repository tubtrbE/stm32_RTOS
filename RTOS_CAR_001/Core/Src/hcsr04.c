/*
 * hcsr04.c
 *
 *  Created on: Sep 4, 2022
 *      Author: ggssg
 */


#include "hcsr04.h"


uint32_t IC_Val1[3] = {0};
uint32_t IC_Val2[3] = {0};

uint32_t Difference[3] = {0};
uint32_t Distance[3]  = {0};
int Is_First_Captured[3] = {0};
float refClock = TIMCLOCK/(PRESCALAR);

void HCSR04_Delay (uint16_t time, TIM_HandleTypeDef *htim)
{
	__HAL_TIM_SET_COUNTER(htim, 0);
	while (__HAL_TIM_GET_COUNTER (htim) < time);

}

void HCSR04_Read (TIM_HandleTypeDef *htim, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{

	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 1);	// pull the TRIG pin HIGH
	HCSR04_Delay(10, htim);  // wait for 10 us
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(htim, TIM_IT_CC1); // enable Interrupt
}


// calculate the distance of HC_SR04
uint32_t HC_SRO4_Dis(TIM_HandleTypeDef *htim, int num) {

	if (Is_First_Captured[num] == 0) // if the first rising edge is not captured
	{
		Is_First_Captured[num] = 1;  // set the first captured as true
		IC_Val1[num] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
//		IC_Val1[num] = htim->Instance->CNT; // read the first value
		IC_Val2[num] = 0;
//		__HAL_TIM_SET_CAPTUREPOLARITY(htim, htim->Channel, TIM_INPUTCHANNELPOLARITY_FALLING);
	}

	else   // If the first rising edge is captured, now we will capture the second edge
	{
		IC_Val2[num] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
		//IC_Val2[num] = htim->Instance->CNT;

		if (IC_Val2[num] > IC_Val1[num])
		{
			Difference[num] = IC_Val2[num]-IC_Val1[num];
		}

		else if (IC_Val1[num] > IC_Val2[num])
		{

			//TIM 1,3,4 is 16bit so overflow is occured when the cnt value is 0xffff
			Difference[num] = (0xffff + IC_Val2[num]) - IC_Val1[num];
		}

//		frequency[num] = refClock/Difference[num];
		Distance[num] = Difference[num]*340/2000;

		//__HAL_TIM_SET_COUNTER(&htim3, 0);  // reset the counter
//		htim->Instance->CNT = 0;

//		__HAL_TIM_SET_CAPTUREPOLARITY(htim, htim->Channel, TIM_INPUTCHANNELPOLARITY_RISING);
		Is_First_Captured[num] = 0; // set it back to false

		//htim is address
		__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
	}
}
