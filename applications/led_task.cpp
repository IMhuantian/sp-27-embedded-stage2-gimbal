#include "cmsis_os.h"
#include "io/led/led.hpp"

// C板 RGB LED: PH10(TIM5_CH1) / PH11(TIM5_CH2) / PH12(TIM5_CH3)
sp::LED led(&htim5);

namespace
{
// 只点亮一个通道（其余两路给 0），三路扫光共用同一个写口
void set_channel(uint8_t channel, float value)
{
  led.set(channel == 0U ? value : 0.0f, channel == 1U ? value : 0.0f, channel == 2U ? value : 0.0f);
}

// 一个通道从 0 渐亮到 1，再渐灭回 0（100 档 × 5 ms = 单程 0.5 s）
void sweep(uint8_t channel)
{
  for (int i = 0; i < 100; i++) {
    set_channel(channel, i * 0.01f);
    osDelay(5);
  }
  for (int i = 100; i > 0; i--) {
    set_channel(channel, i * 0.01f);
    osDelay(5);
  }
}
}  // namespace

// 红、绿、蓝依次渐亮渐灭。
// 灯一直在变说明任务还在被调度，程序卡住的时候灯会停住
extern "C" void led_task()
{
  led.start();

  while (true) {
    sweep(0);
    sweep(1);
    sweep(2);
  }
}
