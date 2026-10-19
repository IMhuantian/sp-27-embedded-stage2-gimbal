#ifndef CAN_DIAG_HPP
#define CAN_DIAG_HPP

#include <cstdint>

#include "can.h"

// 双 CAN 接收诊断路径。
//
// 本模块只接收，不发送 CAN 帧。
//   - CAN1、CAN2 都初始化接收；纯诊断模式不进入正常云台控制/折叠流程。
//   - 不创建 sp::RM_Motor，不调用 cmd()/send()。
//   - FOLD_TEST_ENABLE=1 时，单电机测试复用本模块的接收路径，并通过
//     set_frame_hook() 取 0x205 原始帧；发送由测试模块负责。
//
// CAN_DIAG_ENABLE=1 提供双 CAN 接收；纯诊断在 FOLD_TEST_ENABLE=0 时运行。
#ifndef CAN_DIAG_ENABLE
#define CAN_DIAG_ENABLE 1
#endif

namespace can_diag
{
constexpr uint32_t kOnlineTimeoutMs = 100;
constexpr int kBusCount = 2;
constexpr int kCandidateCount = 11;
constexpr int kUnknownSlots = 16;

// 候选反馈 ID，不据此自动认定型号：
//   bit0..6 : 0x205..0x20B（GM6020 候选反馈 ID）
//   bit7..10: 0x201..0x204（M2006/C610 候选反馈 ID）
constexpr uint16_t kCandidateIds[kCandidateCount] = {
  0x205, 0x206, 0x207, 0x208, 0x209, 0x20A, 0x20B,
  0x201, 0x202, 0x203, 0x204,
};

struct CandidateStats
{
  uint16_t id = 0;
  volatile uint32_t valid_count = 0;    // 标准数据帧 + DLC=8
  volatile uint32_t invalid_count = 0;  // 候选 ID，但 IDE/RTR/DLC 不合法
  volatile uint32_t last_valid_ms = 0;  // 最近有效帧时间；0 = 从未收到
};

struct UnknownIdStats
{
  volatile uint32_t used = 0;
  volatile uint32_t id = 0;
  volatile uint32_t count = 0;
  volatile uint32_t last_ms = 0;
};

struct BusStats
{
  bool init_ok = false;   // 滤波器配置成功
  bool start_ok = false;  // CAN Start + RX0 通知成功

  volatile uint32_t total_frames = 0;    // 所有收到的帧
  volatile uint32_t valid_frames = 0;    // 候选 ID 的有效反馈帧
  volatile uint32_t invalid_frames = 0;  // 候选 ID 的格式错误帧
  volatile uint32_t unknown_frames = 0;  // 非候选 ID / 扩展帧
  volatile uint32_t rx_errors = 0;       // HAL_CAN_GetRxMessage 失败
  volatile uint32_t last_id = 0;         // 最后收到的原始 ID（标准或扩展）
  volatile uint32_t last_unknown_id = 0;
  volatile uint32_t last_frame_ms = 0;
  volatile uint32_t unknown_overflow = 0;

  CandidateStats candidates[kCandidateCount];
  UnknownIdStats unknown[kUnknownSlots];
};

// 0 = 未初始化；1 = 两路都启动；2 = 初始化/启动失败；3 = 接收错误。
extern volatile uint32_t state;
extern BusStats buses[kBusCount];

// 最近一个未知帧（用于调试；14 通道里显示 ID）。
extern volatile uint32_t last_unknown_id;
extern volatile uint32_t last_unknown_bus;

// 每帧原始视图，供单电机测试挂接（不改变诊断统计）。
struct Frame
{
  int bus;  // 0 = CAN1, 1 = CAN2
  uint32_t id;
  bool standard;
  bool remote;
  uint8_t dlc;
  uint8_t * data;
  uint32_t now_ms;
};

using FrameHook = void (*)(const Frame & frame);

// 在 can_diag_start() 之前设置；用于把 0x205 原始帧交给测试模块。
void set_frame_hook(FrameHook hook);

// 初始化并启动 CAN1/CAN2 接收；返回两路是否都启动成功。
bool can_diag_start();

// 候选在线位图（bit0~10）。
uint32_t online_mask(int bus, uint32_t now_ms);

// 只接收诊断主循环；不返回。plot 为 14 通道 SerialPlot 缓冲。
void can_diag_loop(float * plot);

// 收到 CAN RX0 中断时调用；由 hcan 实际外设分发到 CAN1/CAN2。
void can_diag_on_rx(CAN_HandleTypeDef * hcan);
}  // namespace can_diag

#endif  // CAN_DIAG_HPP
