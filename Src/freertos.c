/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stdbool.h"
#include "string.h"

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

extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart3;

uint8_t rx_data;
uint8_t cmd[4] = { 0, };
uint16_t sp_dutycycle;
uint16_t sp_dutycycle2;
uint32_t sensor_val[2];		// [0]: gas, [1]: press
uint16_t sp_gpio;
uint16_t gas_adc_stack[10] = { 0, };
uint16_t press_adc_stack[10] = { 0, };
uint32_t sensor_val_avg[2];

bool SEND_DATA_FLAG = false;
bool SP_FLAG = false;

/* USER CODE END Variables */
osThreadId spTaskHandle;
osThreadId uartRxTaskHandle;
osThreadId sensorTaskHandle;
osThreadId adcstackTaskHandle;
osMutexId rxflagMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint16_t sp_map(uint16_t x, uint16_t y, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
void quick_sort (uint16_t *data, int start, int end);
   
/* USER CODE END FunctionPrototypes */

void StartSpTask(void const * argument);
void StartUartRxTask(void const * argument);
void StartSensorTask(void const * argument);
void StartadcstackTask(void const * argument);

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
  HAL_ADC_Start_DMA(&hadc1, sensor_val, 2);
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of rxflagMutex */
  osMutexDef(rxflagMutex);
  rxflagMutexHandle = osMutexCreate(osMutex(rxflagMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of spTask */
  osThreadDef(spTask, StartSpTask, osPriorityNormal, 0, 128);
  spTaskHandle = osThreadCreate(osThread(spTask), NULL);

  /* definition and creation of uartRxTask */
  osThreadDef(uartRxTask, StartUartRxTask, osPriorityIdle, 0, 128);
  uartRxTaskHandle = osThreadCreate(osThread(uartRxTask), NULL);

  /* definition and creation of sensorTask */
  osThreadDef(sensorTask, StartSensorTask, osPriorityIdle, 0, 128);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

  /* definition and creation of adcstackTask */
  osThreadDef(adcstackTask, StartadcstackTask, osPriorityIdle, 0, 128);
  adcstackTaskHandle = osThreadCreate(osThread(adcstackTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartSpTask */
/**
  * @brief  Function implementing the spTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartSpTask */
void StartSpTask(void const * argument)
{
  /* USER CODE BEGIN StartSpTask */

  /* Infinite loop */
  for(;;)
  {
		if (SP_FLAG) {
			sp_gpio = (sp_map(sp_dutycycle, sp_dutycycle2, 0, 1023, 0, 1023)); 
			int a[10]={0};
			int	i;

			for(i=0; sp_gpio>0; i++)
			{
				a[i] = sp_gpio%2;
				sp_gpio = sp_gpio/2;
			}
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, a[0]);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, a[1]);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, a[2]);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, a[3]);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, a[4]);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, a[5]);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, a[6]);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, a[7]);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, a[8]);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, a[9]);
			SP_FLAG = false;
		}
    osDelay(1);
  }
  /* USER CODE END StartSpTask */
}

/* USER CODE BEGIN Header_StartUartRxTask */
/**
* @brief Function implementing the uartRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartRxTask */
void StartUartRxTask(void const * argument)
{
  /* USER CODE BEGIN StartUartRxTask */
	
	HAL_UART_Receive_IT(&huart3, &rx_data, 1);
	
  /* Infinite loop */
  for(;;)
  {
		if (osSemaphoreWait(rxflagMutexHandle, osWaitForever) == osOK) {
			if (cmd[0] == 0x55) {
				if (cmd[1] == 0xF2) {
					SEND_DATA_FLAG = true;
					memset(cmd, 0, sizeof(cmd));
				}
				else if (cmd[1] == 0xF3) {
					sp_dutycycle = cmd[2];
					sp_dutycycle2 = cmd[3];
					SP_FLAG = true;
					memset(cmd, 0, sizeof(cmd));
				}
			}
			osSemaphoreRelease(rxflagMutexHandle);
		}
    osDelay(1);
  }
  /* USER CODE END StartUartRxTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
	
	uint8_t tx_buf[6];
	tx_buf[0] = 0x55;
	tx_buf[1] = 0xF1;
	
  /* Infinite loop */
  for(;;)
  {
		if (SEND_DATA_FLAG) {
			tx_buf[2] = sensor_val_avg[0] / 256;
			tx_buf[3] = sensor_val_avg[0] % 256;
			tx_buf[4] = sensor_val_avg[1] / 256;
			tx_buf[5] = sensor_val_avg[1] % 256;
			HAL_UART_Transmit(&huart3, tx_buf, 6, 10);
			SEND_DATA_FLAG = false;
#ifdef DEBUG
			printf("gas_val: %d, press_val: %d\r\n", sensor_val[0], sensor_val[1]);
#endif
		}
    osDelay(1);
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartadcstackTask */
/**
* @brief Function implementing the adcstackTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartadcstackTask */
void StartadcstackTask(void const * argument)
{
  /* USER CODE BEGIN StartadcstackTask */
  /* Infinite loop */
  for(;;)
  {
		for(int i=0; i<10; i++)
		{
			gas_adc_stack[i] = gas_adc_stack[i+1];
			press_adc_stack[i] = press_adc_stack[i+1];
			if (i==9)
			{
				gas_adc_stack[i] = sensor_val[0];
				press_adc_stack[i] = sensor_val[1];
			}
		}
		
		quick_sort(gas_adc_stack, 0, 9);
		quick_sort(press_adc_stack, 0, 9);
		
		int sum[2] = {0,0};
		for(int i=2; i<8; i++)
		{
			sum[0] += gas_adc_stack[i];
			sum[1] += press_adc_stack[i];
		}
		sensor_val_avg[0] = sum[0]/6;
		sensor_val_avg[1] = sum[1]/6;
		
    osDelay(1);
  }
  /* USER CODE END StartadcstackTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

uint16_t sp_map(uint16_t x, uint16_t y, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
	x = (x*256) + y;
	return (((x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min + 0.5;
}

void quick_sort (uint16_t *data, int start, int end) {
    if(start >= end) return;
    
    int pivot = start;
    int i = pivot + 1;
    int j = end;
    uint16_t temp;

    while(i <= j) {
        while(i <= end && data[i] <= data[pivot]) i++;
        while(j > start && data[j] >= data[pivot]) j--;
        if(i > j) {
            temp = data[j];
            data[j] = data[pivot];
            data[pivot] = temp;
        } else {
            temp = data[i];
            data[i] = data[j];
            data[j] = temp;
        }
    }
    quick_sort(data, start, j - 1);
    quick_sort(data, j + 1, end);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART3)	{
		if (osSemaphoreWait(rxflagMutexHandle, osWaitForever) == osOK) {
			cmd[0] = cmd[1];
			cmd[1] = cmd[2];
			cmd[2] = cmd[3];
			cmd[3] = rx_data;
			osSemaphoreRelease(rxflagMutexHandle);
		}
		HAL_UART_Receive_IT(&huart3, &rx_data, 1);
	}
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
