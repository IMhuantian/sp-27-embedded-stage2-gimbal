#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

// 波形走 USART1（丝印 UART2）921600，上位机用 SerialPlot。
// 一帧 = AA BB + size + N 个 float；17 通道时 size = 0x44，共 3 + 68 = 71 字节。
// SerialPlot 的 # Channels 必须与之相同（17），否则帧会错位。
// huart1 上只能挂一个 Plotter，两个任务同时发帧会交错成乱码。
//
// 通道（云台 C 板，显示号 = 这里的编号 + 1）：
//   0  测距(3 点中值后) cm        ← -1 = 无效(超时 / 帧错 / 无效值 / 超量程)
//   1  测距状态码 0 正常 / 1 无帧 / 2 超时 / 3 帧错 / 4 无效值或超量程
//   2/3/4  IMU yaw / pitch / roll rad（原始值）
//   5  偏航参考角速度 或 折叠转矩(FO
//      注: FOLD_TORQUE_PLOT 为真时这里是折叠输出转矩)
//   6  折叠目标角 rad（已限速规划）  7  折叠实际角 rad
//   8  俯仰参考角 rad                9  俯仰实际角 rad（仰为正）
//   10 俯仰输出 N·m                 11 失能原因码 0 正常/1 下档/2 遥控失联/3 偏航反馈/4 俯仰反馈/5 姿态联锁
//   12 反馈帧位图（bit0..10 = 0x201..0x20B）  13 拨杆档位 0 无数据/1 下/2 中/3 上
//   14 yaw 命中率(组委会评分 §7.4.1)  15 pitch 命中率  16 当前得分 0~40
// (规则书 4.4.2.2 要求的"目标值 / 反馈值 / 控制量"三件套在 8/9/10 通道)
extern float plot_ch[17];  // 定义在 applications/gimbal_task.cpp

sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (true) {
    // ⚠️ 必须确认上一帧已经发完(UART 回到 READY)再填下一帧：
    // Plotter 内部只有一份 plot_frame_ 缓冲，而发送走 DMA —— 在 DMA 还在读这块内存时改写，
    // SerialPlot 上就会出现错位帧 / 花帧(库里的 send() 只是把 BUSY 记进 hal_status_)。
    // huart1.gState 由 HAL 在 DMA 发送完成中断里恢复成 HAL_UART_STATE_READY。
    // 一帧 = 71 B @921600 ≈ 0.77 ms，正常 1 ms 周期够用；万一上一帧还没发完，本帧直接丢弃(不补发)。
    if (huart1.gState == HAL_UART_STATE_READY) {
      plotter.plot(
        plot_ch[0], plot_ch[1], plot_ch[2], plot_ch[3], plot_ch[4], plot_ch[5], plot_ch[6],
        plot_ch[7], plot_ch[8], plot_ch[9], plot_ch[10], plot_ch[11], plot_ch[12], plot_ch[13],
        plot_ch[14], plot_ch[15], plot_ch[16]);
    }
    osDelay(1);
  }
}
