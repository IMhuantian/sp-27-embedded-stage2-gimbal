#include "cmsis_os.h"
#include "tof_task.hpp"
#include "usart.h"

// 云台板不再使用 DT7/DBus 接收机；本文件只保留 GY-56 的 USART6 回调转发。
extern "C" void uart_task()
{
  while (true) {
    osDelay(10);
  }
}

// GY-56 接 USART6；空闲/收满回调把长度交给 tof_task。
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart->RxEventType == HAL_UART_RXEVENT_HT) return;  // 半满不是完整帧

  if (huart == &huart6) {
    tof_handle_rx(Size);
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart6) {
    tof_handle_error();
  }
}
