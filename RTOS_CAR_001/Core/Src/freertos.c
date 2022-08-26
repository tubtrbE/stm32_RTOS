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
#include "motor.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId     Task1Handle;
osThreadId     Task2Handle;
uint8_t rx_data[2];


/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osMessageQId UartQueueHandle;
osSemaphoreId UartSemaHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart6, &ch, 1, 1000);
    return ch;
}
void task1 (void const * argument);
void task2 (void const * argument);
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
  osMessageQDef(UartQueue, 5, uint8_t);
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
  osThreadDef(tasktest1, task1, osPriorityNormal, 0,configMINIMAL_STACK_SIZE*1);
  Task1Handle = osThreadCreate(osThread(tasktest1), NULL);
  if(!Task1Handle)
	  printf("ERR : Console Task Creation Failure !\r\n");

  osThreadDef(tasktest2, task2, osPriorityNormal, 0,configMINIMAL_STACK_SIZE*1);
  Task2Handle = osThreadCreate(osThread(tasktest2), NULL);

  if(!Task2Handle)
     printf("ERR : CLI Task Creation Failure !\r\n");

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
    BaseType_t ret = pdTRUE;
    signed char cByteRxed = '\0';

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
void task1 (void const * argument)
{
    /* Infinite loop */
    for(;;)
    {
    	HAL_UART_Receive_IT(&huart6, &rx_data[0], 1);
    	osDelay(10);
    }
}
void task2 (void const * argument)
{
    unsigned short cnt=0;

    /* Infinite loop */
    for(;;)
    {
  	  printf("Task 2 is running ======> (%d)\n", cnt++);
         osDelay(3000);
    }
}

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



  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
}

/* USER CODE END Application */
