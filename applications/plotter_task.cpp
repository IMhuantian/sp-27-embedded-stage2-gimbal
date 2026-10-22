#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

// 波形走 USART1（丝印 UART2）921600，上位机用 SerialPlot。
// Current fixed page is documented in docs/TRAJECTORY_SCORE.md (18 channels).
// 一帧 = AA BB + size + N 个 float，size = 18 x 4 = 0x48，共 75 字节；14..17为评分通道。
// huart1 上只能挂一个 Plotter，两个任务同时发帧会交错成乱码。
//
// 通道（云台 C 板；4.4.2.1 要求"测距结果可通过调试输出验证"）：
//   0  测距(3 点中值后) cm        ← -1 = 无效(超时 / 帧错 / 无效值 / 超量程)
//   1  测距状态码 0 正常 / 1 无帧 / 2 超时 / 3 帧错 / 4 无效值或超量程
//   2/3/4  IMU yaw / pitch / roll rad
//   5  偏航目标角速度 rad/s        6  偏航实际角速度 rad/s      7  偏航控制量 V
//   8  俯仰目标角 rad              9  俯仰实际角(IMU) rad      10 俯仰控制量 V
//   11 折叠状态码 0 IDLE / 1 FOLDING / 2 FOLDED / 3 UNFOLDING / 4 FAULT
//   12 折叠电机目标角 rad          13 折叠电机是否被驱动(0/1)
// (规则书 4.4.2.2 要求的"目标值 / 反馈值 / 控制量"三件套对应 5-10 通道)
extern float plot_ch[18];  // 定义在 applications/gimbal_task.cpp

sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (true) {
    // ⚠️ 必须确认上一帧已经发完(UART 回到 READY)再填下一帧：
    // Plotter 内部只有一份 plot_frame_ 缓冲，而发送走 DMA —— 在 DMA 还在读这块内存时改写，
    // SerialPlot 上就会出现错位帧 / 花帧(库里的 send() 只是把 BUSY 记进 hal_status_)。
    // huart1.gState 由 HAL 在 DMA 发送完成中断里恢复成 HAL_UART_STATE_READY。
    // 一帧 = 75 B @921600 ≈ 0.814 ms，正常 1 ms 周期够用；万一上一帧还没发完，本帧直接丢弃(不补发)。
    if (huart1.gState == HAL_UART_STATE_READY) {
      float snapshot[18];
      taskENTER_CRITICAL();
      for (unsigned i = 0; i < 18; ++i) snapshot[i] = plot_ch[i];
      taskEXIT_CRITICAL();
      plotter.plot(
        snapshot[0], snapshot[1], snapshot[2], snapshot[3], snapshot[4], snapshot[5], snapshot[6],
        snapshot[7], snapshot[8], snapshot[9], snapshot[10], snapshot[11], snapshot[12], snapshot[13],
        snapshot[14], snapshot[15], snapshot[16], snapshot[17]);
    }
    osDelay(5);  // 200 Hz telemetry; control and score tasks remain at 1 kHz.
  }
}
