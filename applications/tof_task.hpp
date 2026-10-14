#pragma once

#include <stdint.h>

// GY-56 红外测距对外接口 (规则书 4.4.2.1)
//
// 实现: applications/tof_task.cpp
//       tof_handle_rx() / tof_handle_error() 由 uart_task.cpp 的串口回调(huart6 分支)调用,
//       因为 HAL_UARTEx_RxEventCallback / HAL_UART_ErrorCallback 全工程只能有一份定义。
//       ⚠️ 只有空闲(IDLE)/收满(TC)事件才代表"一批数据收完了": DMA 半满(HT)事件在 uart_task.cpp
//          里已经被过滤掉 —— 那时 DMA 还在往缓冲里写, 解析到的是半帧。
//
// 使用: 云台自动折叠逻辑(4.4.7)请调 tof_get_distance(); 调试/绘图直接读下面的全局量。

// ===== 总开关 =====
// false = 只编译、不接传感器、不挂 DMA(悬空的 USART6_RX 不会产生噪声中断, 也不会有假数据)。
// true  = 正常测距(2026/10/14 底盘部分收尾时打开; 规则书 4.4.2.1 与 7.3 要求验收前必须为 true)。
constexpr bool TOF_ENABLE = true;

// 状态码: 也是 SerialPlot 通道 13 —— 验收时演示"无效值 / 超量程 / 通信异常"处理的依据
enum TofStatus : uint8_t
{
  TOF_STATUS_OK = 0,        // 有量程内的有效帧
  TOF_STATUS_NO_FRAME = 1,  // 开机后还没收到过有效帧
  TOF_STATUS_STALE = 2,     // 通信异常: 超过超时时间没有新有效帧
  TOF_STATUS_FRAME = 3,     // 最近一段数据帧头/长度/校验错(无效帧)
  TOF_STATUS_RANGE = 4,     // 距离读数无效(0)或超量程
};

// uart_task.cpp 的回调转发入口
void tof_handle_rx(uint16_t size);  // 只在空闲/收满事件里调用
void tof_handle_error();

// ===== 测距数据(volatile: 中断里写, 任务里读) =====
extern volatile float tof_distance_cm;  // 中值滤波后的距离, 单位 cm, <0 = 无效(折叠逻辑用这个)
extern volatile float tof_raw_cm;       // 最近一帧(量程内)的原始距离, 单位 cm, <0 = 无
extern volatile bool tof_valid;         // 当前数据是否可用
extern volatile uint8_t tof_status;     // TofStatus
extern volatile uint8_t tof_mode;       // 帧里的测量模式: 0 长距离/1 快速/2 高精度/3 一般, 0xFF=未知
extern volatile int8_t tof_temp_c;      // 帧里的模块温度 ℃, <0 = 该帧没带温度
extern volatile uint32_t tof_last_ms;   // 最近一次有效帧的时刻, ms

// ===== 计数(验收时演示"作基本处理") =====
extern volatile uint32_t tof_frames;     // 有效帧数
extern volatile uint32_t tof_bad_sum;    // 帧头/长度/校验错计数
extern volatile uint32_t tof_out_range;  // 读数无效(0)或超量程计数
extern volatile uint32_t tof_timeouts;   // 通信超时次数
extern volatile uint32_t tof_errors;     // 串口错误(帧错误/溢出等)次数

// 给云台自动折叠逻辑(4.4.7)的统一入口: 有新鲜的有效数据时返回 true, 并把距离(单位 cm)写入 *cm
bool tof_get_distance(float* cm, uint32_t now_ms);
