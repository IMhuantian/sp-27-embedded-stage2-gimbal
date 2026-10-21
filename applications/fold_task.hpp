#ifndef FOLD_TASK_HPP
#define FOLD_TASK_HPP

#include <cstdint>

// ============================ 4.4.7 云台自动折叠（框架）============================
// 规则书要求：
//   - 用独立的 FreeRTOS 任务完成折叠决策与动作；
//   - 用测距结果 + 阈值/回差 + 状态判断，避免在阈值附近频繁切换；
//   - 有限位 / 超时 / 异常退出保护。
//
// 本轮状态（2026-10-14，云台机构与折叠电机尚未接线）：
//   FOLD_ENABLE = false → fold_update() 只把状态机停在 IDLE，不给任何电机指令；
//   gimbal_task.cpp 因此始终给折叠电机 0（并清掉位置环积分）。
//   接线后要做的事见 README「4.4.7 待做」：量 FOLD_STOW_RAD / FOLD_DEPLOY_RAD、
//   接限位开关、按隧道实测整定 FOLD_TRIGGER_CM / FOLD_RELEASE_CM、把 FOLD_ENABLE 改成 true。

constexpr bool FOLD_ENABLE = true;

// 阈值：先给保守值，必须按隧道实测整定（回差 ≥ 3 cm 是 GY-56 SET/OUT 的固有回差量级）
// ---- 时序（按底盘自动行驶速度推算；改车速或折叠速度时这几个数要一起改）----
// 底盘自动行驶速度（can_task.cpp 的 AUTO_SPEED_MPS），云台这边只能按设计值推算
constexpr float CHASSIS_SPEED_MPS = 0.6f;
// 折叠动作时间：1.2 rad 行程、上限 1.2 rad/s、加减速 3 rad/s^2 的梯形规划 ≈ 1.4 s
constexpr float FOLD_MOVE_TIME_S = 1.4f;
constexpr uint32_t FOLD_MOVE_TIME_MS = 1400u;
// 触发余量：去抖 300 ms + 采样/刹停余量
constexpr float FOLD_TRIGGER_MARGIN_CM = 25.0f;
// 触发距离 = 车速 × 动作时间 + 余量，保证折叠在进隧道口之前做完（0.6×1.4×100+25 ≈ 109 cm）
constexpr float FOLD_TRIGGER_CM =
  CHASSIS_SPEED_MPS * FOLD_MOVE_TIME_S * 100.0f + FOLD_TRIGGER_MARGIN_CM;
// 展开阈值：比触发阈值大（回差），又不能超过 GY-56 量程 120 cm
constexpr float FOLD_RELEASE_CM = 115.0f;
// 收折后至少保持这么久再允许展开（隧道长度 / 车速：3 s ≈ 1.8 m @0.6 m/s）
constexpr uint32_t FOLD_MIN_HOLD_MS = 3000u;
constexpr uint32_t FOLD_DEBOUNCE_MS = 300;       // 条件连续满足这么久才切状态
constexpr uint32_t FOLD_MOVE_TIMEOUT_MS = 3000;  // 动作超时 → FAULT 并停机
static_assert(
  FOLD_TRIGGER_CM < 120.0f, "触发距离超出 GY-56 量程 120cm：折叠来不及做完，加大折叠速度");
static_assert(FOLD_RELEASE_CM > FOLD_TRIGGER_CM, "释放阈值必须大于触发阈值（回差）");
constexpr float FOLD_STOW_RAD = 0.0f;    // 收折位置（相对上电位置，待实测）
constexpr float FOLD_DEPLOY_RAD = 1.2f;  // 展开位置（相对上电位置，待实测）

// 台架单独测折叠电机：AUTO 按测距自动决策；FOLD / UNFOLD 强制收 / 展（不看测距）
enum class FoldCmd : uint8_t
{
  AUTO = 0,
  FOLD = 1,
  UNFOLD = 2
};
constexpr FoldCmd FOLD_CMD = FoldCmd::AUTO;

enum class FoldState : uint8_t
{
  IDLE = 0,       // 不动作（未使能 / 条件不满足）
  FOLDING = 1,    // 正在收折
  FOLDED = 2,     // 已收折
  UNFOLDING = 3,  // 正在展开
  FAULT = 4       // 超时 / 异常，已停机，需要重新上电或复位
};

// 折叠决策（在 FoldTask 里周期调用，周期建议 10 ms）
void fold_update(uint32_t now_ms);

// 折叠电机的目标角（rad，相对上电位置）。
// 返回 false = 现在不该驱动折叠电机（未使能 / 未到动作条件 / 已到位 / FAULT）。
bool fold_target_rad(float * target_rad);

#endif  // FOLD_TASK_HPP
