/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    CAN_HandleTypeDef *hcan;
    uint32_t id;
    uint8_t len;
    uint8_t *buf_data;
}can_device_transmit_member;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for CAN1_Sending_Service */
osThreadId_t CAN1_Sending_ServiceHandle;
const osThreadAttr_t CAN1_Sending_Service_attributes = {
  .name = "CAN1_Sending_Service",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CAN2_Sending_Service */
osThreadId_t CAN2_Sending_ServiceHandle;
const osThreadAttr_t CAN2_Sending_Service_attributes = {
  .name = "CAN2_Sending_Service",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for DJI_Motor_Service */
osThreadId_t DJI_Motor_ServiceHandle;
const osThreadAttr_t DJI_Motor_Service_attributes = {
  .name = "DJI_Motor_Service",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LED_TASK */
osThreadId_t LED_TASKHandle;
const osThreadAttr_t LED_TASK_attributes = {
  .name = "LED_TASK",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DJI_MOTOR_DRIVER */
osThreadId_t DJI_MOTOR_DRIVERHandle;
const osThreadAttr_t DJI_MOTOR_DRIVER_attributes = {
  .name = "DJI_MOTOR_DRIVER",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for POWER_RECEIVER */
osThreadId_t POWER_RECEIVERHandle;
const osThreadAttr_t POWER_RECEIVER_attributes = {
  .name = "POWER_RECEIVER",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for REMOTE_CONTROLER */
osThreadId_t REMOTE_CONTROLERHandle;
const osThreadAttr_t REMOTE_CONTROLER_attributes = {
  .name = "REMOTE_CONTROLER",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for can1_send_fifo */
osMessageQueueId_t can1_send_fifoHandle;
const osMessageQueueAttr_t can1_send_fifo_attributes = {
  .name = "can1_send_fifo"
};
/* Definitions for can2_send_fifo */
osMessageQueueId_t can2_send_fifoHandle;
const osMessageQueueAttr_t can2_send_fifo_attributes = {
  .name = "can2_send_fifo"
};
/* Definitions for CAN1CountingSem */
osSemaphoreId_t CAN1CountingSemHandle;
const osSemaphoreAttr_t CAN1CountingSem_attributes = {
  .name = "CAN1CountingSem"
};
/* Definitions for CAN2CountingSem */
osSemaphoreId_t CAN2CountingSemHandle;
const osSemaphoreAttr_t CAN2CountingSem_attributes = {
  .name = "CAN2CountingSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void can1_sending_service(void *argument);
void can2_sending_service(void *argument);
void DJI_motor_service(void *argument);
void led_task(void *argument);
void dji_motor_driver(void *argument);
void power_receiver(void *argument);
void remote_controler(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of CAN1CountingSem */
  CAN1CountingSemHandle = osSemaphoreNew(3, 3, &CAN1CountingSem_attributes);

  /* creation of CAN2CountingSem */
  CAN2CountingSemHandle = osSemaphoreNew(3, 3, &CAN2CountingSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of can1_send_fifo */
  can1_send_fifoHandle = osMessageQueueNew (16, sizeof(can_device_transmit_member), &can1_send_fifo_attributes);

  /* creation of can2_send_fifo */
  can2_send_fifoHandle = osMessageQueueNew (16, sizeof(can_device_transmit_member), &can2_send_fifo_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of CAN1_Sending_Service */
  CAN1_Sending_ServiceHandle = osThreadNew(can1_sending_service, NULL, &CAN1_Sending_Service_attributes);

  /* creation of CAN2_Sending_Service */
  CAN2_Sending_ServiceHandle = osThreadNew(can2_sending_service, NULL, &CAN2_Sending_Service_attributes);

  /* creation of DJI_Motor_Service */
  DJI_Motor_ServiceHandle = osThreadNew(DJI_motor_service, NULL, &DJI_Motor_Service_attributes);

  /* creation of LED_TASK */
  LED_TASKHandle = osThreadNew(led_task, NULL, &LED_TASK_attributes);

  /* creation of DJI_MOTOR_DRIVER */
  DJI_MOTOR_DRIVERHandle = osThreadNew(dji_motor_driver, NULL, &DJI_MOTOR_DRIVER_attributes);

  /* creation of POWER_RECEIVER */
  POWER_RECEIVERHandle = osThreadNew(power_receiver, NULL, &POWER_RECEIVER_attributes);

  /* creation of REMOTE_CONTROLER */
  REMOTE_CONTROLERHandle = osThreadNew(remote_controler, NULL, &REMOTE_CONTROLER_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_can1_sending_service */
/**
  * @brief  Function implementing the CAN1_Sending_Service thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_can1_sending_service */
__weak void can1_sending_service(void *argument)
{
  /* USER CODE BEGIN can1_sending_service */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END can1_sending_service */
}

/* USER CODE BEGIN Header_can2_sending_service */
/**
* @brief Function implementing the CAN2_Sending_Service thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_can2_sending_service */
__weak void can2_sending_service(void *argument)
{
  /* USER CODE BEGIN can2_sending_service */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END can2_sending_service */
}

/* USER CODE BEGIN Header_DJI_motor_service */
/**
* @brief Function implementing the DJI_Motor_Service thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DJI_motor_service */
__weak void DJI_motor_service(void *argument)
{
  /* USER CODE BEGIN DJI_motor_service */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DJI_motor_service */
}

/* USER CODE BEGIN Header_led_task */
/**
* @brief Function implementing the LED_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_task */
__weak void led_task(void *argument)
{
  /* USER CODE BEGIN led_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END led_task */
}

/* USER CODE BEGIN Header_dji_motor_driver */
/**
* @brief Function implementing the DJI_MOTOR_DRIVER thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dji_motor_driver */
__weak void dji_motor_driver(void *argument)
{
  /* USER CODE BEGIN dji_motor_driver */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END dji_motor_driver */
}

/* USER CODE BEGIN Header_power_receiver */
/**
* @brief Function implementing the POWER_RECEIVER thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_power_receiver */
__weak void power_receiver(void *argument)
{
  /* USER CODE BEGIN power_receiver */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END power_receiver */
}

/* USER CODE BEGIN Header_remote_controler */
/**
* @brief Function implementing the REMOTE_CONTROLER thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_remote_controler */
__weak void remote_controler(void *argument)
{
  /* USER CODE BEGIN remote_controler */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END remote_controler */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

