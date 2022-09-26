/*
 * hcsr04.h
 *
 *  Created on: Sep 4, 2022
 *      Author: ggssg
 */
#include "tim.h"


#ifndef INC_HCSR04_H_
#define INC_HCSR04_H_

#define TIMCLOCK   90000000
#define PRESCALAR  90
#define DANGER_DIS 200

extern uint32_t Distance[3];

void HCSR04_Read (TIM_HandleTypeDef *htim, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HCSR04_Delay (uint16_t time, TIM_HandleTypeDef *htim);
uint32_t HC_SRO4_Dis(TIM_HandleTypeDef *htim, int num);

#endif /* INC_HCSR04_H_ */
