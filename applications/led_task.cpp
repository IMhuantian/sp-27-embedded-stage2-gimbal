#include "cmsis_os.h"
#include "io/led/led.hpp"

// C板 RGB LED: PH10(TIM5_CH1) / PH11(TIM5_CH2) / PH12(TIM5_CH3)
sp::LED led(&htim5);

// 红、绿、蓝依次渐亮渐灭。
// 灯一直在变说明任务还在被调度，程序卡住的时候灯会停住
extern "C" void led_task()
{
  led.start();

  while (true) {
    for (uint8_t i = 0; i < 100; i++) {
      led.set(i * 0.01f, 0, 0);
      osDelay(5);
    }
    for (int i = 100; i > 0; i--) {
      led.set(i * 0.01f, 0, 0);
      osDelay(5);
    }

    for (uint8_t i = 0; i < 100; i++) {
      led.set(0, i * 0.01f, 0);
      osDelay(5);
    }
    for (int i = 100; i > 0; i--) {
      led.set(0, i * 0.01f, 0);
      osDelay(5);
    }

    for (uint8_t i = 0; i < 100; i++) {
      led.set(0, 0, i * 0.01f);
      osDelay(5);
    }
    for (int i = 100; i > 0; i--) {
      led.set(0, 0, i * 0.01f);
      osDelay(5);
    }
  }
}
