#ifndef GM6020_FOLD_TEST_HPP
#define GM6020_FOLD_TEST_HPP

#include <cstdint>

// 单电机 20° 相对往返测试（只控制 CAN1 上 GM6020 ID1，反馈 0x205）。
//
// 电机为 GM6020 电流模式：控制帧 0x1FE，ID1~4；tx 0x1FE / 0x2FE 的选择由
// sp::RM_Motors::GM6020 处理。cmd 参数单位是 N·m，不是 A 或 V。
// 云台无遥控器、无人工触发：本文件默认 FOLD_TEST_ENABLE=0，不参与构建入口；
// 显式改为 1 并重新构建后，上电收到有效 0x205 并稳定 500 ms 才会自动执行一次。
// 自动执行一次 +20° → 停留 → 返回起点；完成后不重复。
// 该符号只表示编码器角度增加，不声称等于物理顺时针/逆时针；
// 上电前必须确认 20° 机械/线束空间，并准备断电。
//
// 该测试不进入正常云台控制/折叠状态机，不使用旧 FOLD_MOTOR_ID=3。
#ifndef FOLD_TEST_ENABLE
#define FOLD_TEST_ENABLE 0
#endif

namespace gm6020_fold_test
{
constexpr uint8_t kMotorId = 1;       // 用户确认：CAN1 GM6020 ID1
constexpr uint16_t kExpectedRxId = 0x205;
constexpr bool kMotorOnCan1 = true;

// 用户选择：编码器正方向 +20°；不声称物理顺/逆时针。
// 仍保留配置合法性检查：只有 enabled=true 且 direction=±1 才允许动作。
constexpr bool kActionEnabled = true;
constexpr int kDirection = +1;  // 编码器正方向

constexpr float kTravelRad = 0.34906585f;  // 20° = 0.34906585 rad

// 保守初值，无负载/实机整定证据；交付中必须明示。
// 低输出电流模式初值，无负载/实机整定证据。
// GM6020 库量程：±16384 raw ↔ ±3 A ↔ ±2.223 N·m；
// 0.25 N·m ≈ 0.337 A ≈ +1842 raw。
constexpr float kMaxTorqueNm = 0.25f;
constexpr float kKp = 1.2f;   // N·m/rad
constexpr float kKd = 0.06f;  // N·m/(rad/s)
constexpr float kMaxRefSpeedRadS = 0.15f;
constexpr float kMaxRefAccelRadS2 = 0.2f;
constexpr float kPosTolRad = 0.03f;
constexpr float kSpeedTolRadS = 0.05f;
constexpr uint32_t kSettleMs = 200;
constexpr uint32_t kHoldMs = 300;
constexpr uint32_t kTimeoutMs = 12000;
// 往/返/停留共用的允许行程余量；范围 [start-0.15, start+20°+0.15]。
constexpr float kTravelMarginRad = 0.15f;
constexpr uint32_t kAutoStartDelayMs = 500;  // 首次有效反馈后等待，再自动开始

enum class State : uint8_t
{
  ConfigLocked = 0,
  WaitFeedback = 1,
  Ready = 2,
  MoveOut = 3,
  Hold = 4,
  MoveBack = 5,
  Done = 6,
  Aborted = 7,
  CanError = 8,
};

enum class Fault : uint8_t
{
  None = 0,
  ConfigLocked = 1,
  NoFeedback = 2,
  FeedbackLost = 3,
  Timeout = 4,
  NonFinite = 5,
  Travel = 6,
  CanError = 7,
};

void gm6020_fold_test_loop(float * plot);
}  // namespace gm6020_fold_test

#endif  // GM6020_FOLD_TEST_HPP
