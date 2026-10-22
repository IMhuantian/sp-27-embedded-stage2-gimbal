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
osThreadId defaultTaskHandle;
osThreadId PlotterTaskHandle;
osThreadId GimbalTaskHandle;
osThreadId FoldTaskHandle;
osThreadId ImuTaskHandle;
osThreadId UartTaskHandle;
osThreadId LedTaskHandle;
osThreadId BuzzerTaskHandle;
osThreadId TofTaskHandle;
osThreadId TrajTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
extern void plotter_task(void const * argument);
extern void gimbal_task(void const * argument);
extern void fold_task(void const * argument);
extern void imu_task(void const * argument);
extern void uart_task(void const * argument);
extern void led_task(void const * argument);
extern void buzzer_task(void const * argument);
extern void tof_task(void const * argument);
extern void traj_task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of PlotterTask */
  osThreadDef(PlotterTask, plotter_task, osPriorityLow, 0, 128);
  PlotterTaskHandle = osThreadCreate(osThread(PlotterTask), NULL);

  /* definition and creation of GimbalTask (云台控制: 1kHz) */
  osThreadDef(GimbalTask, gimbal_task, osPriorityAboveNormal, 0, 256);
  GimbalTaskHandle = osThreadCreate(osThread(GimbalTask), NULL);

  /* definition and creation of FoldTask (4.4.7 折叠决策: 10ms, 本轮只跑骨架) */
  osThreadDef(FoldTask, fold_task, osPriorityNormal, 0, 256);
  FoldTaskHandle = osThreadCreate(osThread(FoldTask), NULL);

  /* definition and creation of ImuTask */
  osThreadDef(ImuTask, imu_task, osPriorityAboveNormal, 0, 512);
  ImuTaskHandle = osThreadCreate(osThread(ImuTask), NULL);

  /* definition and creation of UartTask */
  osThreadDef(UartTask, uart_task, osPriorityNormal, 0, 256);
  UartTaskHandle = osThreadCreate(osThread(UartTask), NULL);

  /* definition and creation of LedTask */
  osThreadDef(LedTask, led_task, osPriorityLow, 0, 128);
  LedTaskHandle = osThreadCreate(osThread(LedTask), NULL);

  /* definition and creation of BuzzerTask */
  osThreadDef(BuzzerTask, buzzer_task, osPriorityLow, 0, 128);
  BuzzerTaskHandle = osThreadCreate(osThread(BuzzerTask), NULL);

  /* definition and creation of TofTask */
  osThreadDef(TofTask, tof_task, osPriorityNormal, 0, 256);
  TofTaskHandle = osThreadCreate(osThread(TofTask), NULL);

  /* definition and creation of TrajTask (4.4.8 轨迹给定, 独立任务 1kHz) */
  osThreadDef(TrajTask, traj_task, osPriorityNormal, 0, 256);
  TrajTaskHandle = osThreadCreate(osThread(TrajTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
