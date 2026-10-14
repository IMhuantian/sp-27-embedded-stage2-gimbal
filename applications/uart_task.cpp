#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "tof_task.hpp"  // ← 新增: applications/ 已在 CMake 的 include 目录里

// DT7 接收机接在 USART3 上(PC10=TX / PC11=RX)，100000bps / 偶校验 / 8 数据位 / 1 停止位，
// 这些都在 cboard_gimbal.ioc 里配好了，接收走 DMA1_Stream1
sp::DBus remote(&huart3);

extern "C" void uart_task()
{
  remote.request();

  while (true) {
    // remote 上的数据：ch_lh / ch_lv / ch_rh / ch_rv / ch_lu 是摇杆和拨轮，范围 [-1, 1]；
    // sw_l / sw_r 是左右三位开关；is_alive() 看接收机在不在线
    osDelay(10);
  }
}

// 串口空闲中断 + DMA 收满，把收到的长度交给对应解析器，然后马上重新挂上 DMA
// ⚠️ 全工程只有这一个定义：所有串口(GY-56 的 USART6)都在这里分支转发
// ⚠️ DMA 半满(HT)事件也会进这个回调(HAL 没有关掉 HT 中断)，但半满不是"一批数据收完了"：
//      那时 DMA 还在往缓冲里写，解析到的是半帧；DBus 那边还会以错误长度提前返回。
//      所以 HT 事件直接忽略，只处理空闲(IDLE)与收满(TC) —— 两者都表示 DMA 已经停下来了。
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart->RxEventType == HAL_UART_RXEVENT_HT) return;  // 半满：不是一批数据收完

  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  } else if (huart == &huart6) {
    tof_handle_rx(Size);  // GY-56: 解析帧 + 重新挂 DMA(在 tof_task.cpp 里)
  }
}

// 收出错的时候重新挂 DMA
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  } else if (huart == &huart6) {
    tof_handle_error();  // GY-56: 重新挂 DMA
  }
}
