#include "cmsis_os.h"
#include "usart.h"
#include "tof_task.hpp"
#include "trajectory_score.hpp"

// USART1（丝印UART2）：460800 8N1，100 Hz；控制和评分仍为1 kHz。
// 7通道：Yaw比例、Pitch比例、Yaw目标/实际(度)、Pitch目标/实际(度)、测距(cm)。
// 内部18通道诊断保留，串口只发送下面7项；参见docs/TELEMETRY_7CH.md。
extern float plot_ch[18];

namespace {
constexpr float RAD_TO_DEG = 57.29577951308232f;
struct __attribute__((packed)) TelemetryFrame {
  uint8_t start[2] = {0xAA, 0xBB};
  uint8_t size = 7u * sizeof(float);
  float values[7] = {};
  uint8_t checksum = 0;
};
static_assert(sizeof(TelemetryFrame) == 32u, "Seven-channel wire frame must be 32 bytes");
TelemetryFrame frame;
}

extern "C" void plotter_task()
{
  while (true) {
    // DMA读取期间不改帧；32字节在460800下约0.694 ms。
    if (huart1.gState == HAL_UART_STATE_READY) {
      taskENTER_CRITICAL();
      frame.values[0] = trajectory_score::latest.yaw_ratio;
      frame.values[1] = trajectory_score::latest.pitch_ratio;
      frame.values[2] = plot_ch[0] * RAD_TO_DEG;
      frame.values[3] = plot_ch[1] * RAD_TO_DEG;
      frame.values[4] = plot_ch[5] * RAD_TO_DEG;
      frame.values[5] = plot_ch[6] * RAD_TO_DEG;
      frame.values[6] = tof_valid ? tof_distance_cm : -1.0f;
      taskEXIT_CRITICAL();
      // SerialPlot固定帧头AA BB 1C，校验仅覆盖28字节浮点载荷。
      frame.checksum = 0;
      const auto * payload = reinterpret_cast<const uint8_t *>(frame.values);
      for (unsigned i = 0; i < sizeof(frame.values); ++i) {
        frame.checksum = static_cast<uint8_t>(frame.checksum + payload[i]);
      }
      HAL_UART_Transmit_DMA(&huart1, reinterpret_cast<uint8_t *>(&frame), sizeof(frame));
    }
    osDelay(10);
  }
}
