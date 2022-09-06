/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "motor.h"
#include "usart.h"
#include "hcsr04.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum {
	UP,
	DOWN
}ODO_STAT;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

//uint8_t odo_flag;
//uint32_t odo_count;

osThreadId     Task1Handle;
osThreadId     Task2Handle;

/*
 * Distance[0], Difference[0] = Left
 * Distance[1], Difference[1] = Front
 * Distance[2], Difference[2] = Right
 * */

uint32_t Dis[3];
uint32_t temp_count[4];


osThreadId     HS_SR04_Left_Checking;
osThreadId     HS_SR04_Front_Checking;
osThreadId     HS_SR04_Right_Checking;

osThreadId     HS_SR04_Left_Handle;
osThreadId     HS_SR04_Front_Handle;
osThreadId     HS_SR04_Right_Handle;
uint8_t rx_data[2];

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osMessageQId UartQueueHandle;
osSemaphoreId UartSemaHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//int __io_putchar(int ch) {
//    HAL_UART_Transmit(&huart3, &ch, 1, 1000);
//    return ch;
//}


void ThreadInit () ;
int odo_adjust (int odo_num, ODO_STAT odo_status);


void CheckingUartReceive (void const * argument);
void CheckingLeft (void const * argument);
void CheckingFront (void const * argument);
void CheckingRight (void const * argument);
/** Car Control Using RasberryPi*/
void odometryTask (void const * argument);
void CarLeftSide (void const * argument);
void CarFrontSide (void const * argument);
void CarRightSide (void const * argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	  Motor_Init();
	  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

	  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
	  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_3);
	  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_4);

	  HAL_UART_Receive_IT(&huart6, &rx_data[0], 1);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of UartSema */
  osSemaphoreDef(UartSema);
  UartSemaHandle = osSemaphoreCreate(osSemaphore(UartSema), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of UartQueue */
  osMessageQDef(UartQueue, 8, uint8_t);
  UartQueueHandle = osMessageCreate(osMessageQ(UartQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  ThreadInit ();
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    BaseType_t xHigherPriorityWasTaken = pdFALSE;
    BaseType_t ret = pdTRUE;      // if semaphore is ret you know that isr give you queue
    signed char cByteRxed = '\0'; // this value is what you receive

  /* Infinite loop */
	for (;;) {

		/* Block until the next char is available. */
		ret = xSemaphoreTakeFromISR(UartSemaHandle, &xHigherPriorityWasTaken);
		if (ret == pdPASS) {
			/* Handle character in QUEUE */
			ret = xQueueReceiveFromISR(UartQueueHandle, &cByteRxed,
					&xHigherPriorityWasTaken);
			if (ret) {
				// do something . . .
				Move(cByteRxed - '0');
			}
		}
		osDelay(50);
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/*
 * Distance[0], Difference[0] = Left
 * Distance[1], Difference[1] = Front
 * Distance[2], Difference[2] = Right
 * */

// Task ---------------------------------------------------------------------------------------
void odometryTask (void const * argument)
{
	for (;;) {


//		uint32_t temp_count[4];

		// check the safety maximun speed
		// and add some Algorithms
		for(int i = 0; i < 4; i++) {
			temp_count[i] = odo_count[i];

			if (odo_count[i] > 200) {
				odo_adjust(i, DOWN);
			}
			else if (odo_count[i] > 200 - 5) {
				odo_adjust(i, UP);
			}

			odo_count[i] = 0;
		}

		osDelay(1000);
	}
}

void CarLeftSide (void const * argument){

	for (;;) {

		osDelay(500);
	}
}

void CarFrontSide (void const * argument){

	for (;;) {

		osDelay(500);
	}
}
void CarRightSide (void const * argument){

	for (;;) {

		osDelay(500);
	}
}


// ISR Checking-------------------------------------------------------------------------------

void CheckingUartReceive (void const * argument)
{
    /* Infinite loop */
    for(;;)
    {
    	HAL_UART_Receive_IT(&huart6, &rx_data[0], 1);
    	osDelay(10);
    }
}
void CheckingLeft (void const * argument) {
    /* Infinite loop */
    for(;;)
    {
    	HCSR04_Read(&htim1, GPIOF, GPIO_PIN_13);

    	osDelay(60);
    }
}
void CheckingFront (void const * argument) {
    /* Infinite loop */
    for(;;)
    {
    	HCSR04_Read(&htim3, GPIOA, GPIO_PIN_5);
    	osDelay(60);
    }
}
void CheckingRight (void const * argument) {
    /* Infinite loop */
    for(;;)
    {
    	HCSR04_Read(&htim4, GPIOD, GPIO_PIN_13);
    	osDelay(60);
    }
}

// CallBack Session
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
	char *pErrStr = "ERR : QTx Fail!\r\n";
	// typedef long BaseType_t;
	BaseType_t ret = pdTRUE;
	//#define portBASE_TYPE	long
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if(huart->Instance == USART6) {

		ret = xQueueSendFromISR(UartQueueHandle, &rx_data[0], &xHigherPriorityTaskWoken );
		if(ret) {
			xSemaphoreGiveFromISR( UartSemaHandle, &xHigherPriorityTaskWoken );
		}
		else {
			HAL_UART_Transmit(&huart6, (uint8_t*)pErrStr, strlen(pErrStr), 0xffff);
		}

		HAL_UART_Receive_IT(&huart6, &rx_data[0], 1);

	}
//	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );


  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//Checking the left Distance
	if (htim->Instance == TIM1) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			HC_SRO4_Dis(htim, 0);
		}
	}

	//Checking the Front Distance
	if (htim->Instance == TIM3) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			HC_SRO4_Dis(htim, 1);
		}
	}

	//Checking the Right Distance
	if (htim->Instance == TIM4) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			HC_SRO4_Dis(htim, 2);
		}
	}

	//using the general purpose timer because i will use checking the signal only
	if (htim->Instance == TIM8) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			if (odo_flag[0] == 1) {
				odo_count[0]++;
			}
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			if (odo_flag[1] == 1) {
				odo_count[1]++;
			}
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			if (odo_flag[2] == 1) {
				odo_count[2]++;
			}
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			if (odo_flag[3] == 1) {
				odo_count[3]++;
			}
		}
	}
}

void ThreadInit () {
	  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);


	  osThreadDef(UartCheck, CheckingUartReceive, osPriorityNormal, 0,configMINIMAL_STACK_SIZE*1);
	  Task1Handle = osThreadCreate(osThread(UartCheck), NULL);
	  if(!Task1Handle)
		  printf("ERR : Console Task Creation Failure !\r\n");

	  osThreadDef(UartTask, odometryTask, osPriorityNormal, 0,configMINIMAL_STACK_SIZE*1);
	  Task2Handle = osThreadCreate(osThread(UartTask), NULL);

	  if(!Task2Handle)
	     printf("ERR : CLI Task Creation Failure !\r\n");

	  // HC-SR04 LEFT -------------------------------------------------------------------------------------------------------
	  osThreadDef(LeftCheck, CheckingLeft, osPriorityNormal, 0,configMINIMAL_STACK_SIZE*1);
	  HS_SR04_Left_Checking = osThreadCreate(osThread(LeftCheck), NULL);
	  if(!HS_SR04_Left_Checking)
		  printf("ERR : HS_SR04_left_Checking Creation Failure !\r\n");

	  osThreadDef(LeftTask, CarLeftSide, osPriorityNormal, 0,configMINIMAL_STACK_SIZE*1);
	  HS_SR04_Left_Handle = osThreadCreate(osThread(LeftTask), NULL);
	  if(!HS_SR04_Left_Handle)
		  printf("ERR : HS_SR04_left_Handle Creation Failure !\r\n");

	  // HC-SR04 FRONT -------------------------------------------------------------------------------------------------------
	  osThreadDef(FrontCheck, CheckingFront, osPriorityNormal, 0,configMINIMAL_STACK_SIZE*1);
	  HS_SR04_Front_Checking = osThreadCreate(osThread(FrontCheck), NULL);
	  if(!HS_SR04_Front_Checking)
		  printf("ERR : HS_SR04_Front_Checking Creation Failure !\r\n");

	  osThreadDef(FrontTask, CarFrontSide, osPriorityNormal, 0,configMINIMAL_STACK_SIZE*1);
	  HS_SR04_Front_Handle = osThreadCreate(osThread(FrontTask), NULL);
	  if(!HS_SR04_Front_Handle)
		  printf("ERR : HS_SR04_Front_Handle Creation Failure !\r\n");

	  // HC-SR04 RIGHT -------------------------------------------------------------------------------------------------------
	  osThreadDef(RightCheck, CheckingRight, osPriorityNormal, 0,configMINIMAL_STACK_SIZE*1);
	  HS_SR04_Right_Checking = osThreadCreate(osThread(RightCheck), NULL);
	  if(!HS_SR04_Right_Checking)
		  printf("ERR : HS_SR04_Right_Checking Creation Failure !\r\n");

	  osThreadDef(RightTask, CarRightSide, osPriorityNormal, 0,configMINIMAL_STACK_SIZE*1);
	  HS_SR04_Right_Handle = osThreadCreate(osThread(RightTask), NULL);
	  if(!HS_SR04_Right_Handle)
		  printf("ERR : HS_SR04_Right_Handle Creation Failure !\r\n");
}



//Func
int odo_adjust (int odo_num, ODO_STAT odo_status) {
	//		TIM2->CCR4++;//odo_count[0]
	//		TIM2->CCR3++;//odo_count[1]
	//		TIM2->CCR1++;//odo_count[2]
	//		TIM2->CCR2++;//odo_count[3]


	if (odo_num == 0 || odo_status == UP) {
		TIM2->CCR4++;
		return 1;
	}
	else if (odo_num == 0 || odo_status == DOWN) {
		TIM2->CCR4--;
		return 2;
	}
//---------------------------------------------------------------------
	if (odo_num == 1 || odo_status == UP) {
		TIM2->CCR3++;
		return 1;
	}
	else if (odo_num == 1 || odo_status == DOWN) {
		TIM2->CCR3--;
		return 2;
	}
//---------------------------------------------------------------------
	if (odo_num == 2 || odo_status == UP) {
		TIM2->CCR1++;
		return 1;
	}
	else if (odo_num == 2 || odo_status == DOWN) {
		TIM2->CCR1--;
		return 2;
	}
//---------------------------------------------------------------------
	if (odo_num == 3 || odo_status == UP) {
		TIM2->CCR2++;
		return 1;
	}
	else if (odo_num == 3 || odo_status == DOWN) {
		TIM2->CCR2--;
		return 2;
	}
//---------------------------------------------------------------------
	return 0;
}



/* USER CODE END Application */
