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

constexpr bool FOLD_ENABLE = false;

// 阈值：先给保守值，必须按隧道实测整定（回差 ≥ 3 cm 是 GY-56 SET/OUT 的固有回差量级）
constexpr float FOLD_TRIGGER_CM = 60.0f;        // 距离小于它就收折
constexpr float FOLD_RELEASE_CM = 75.0f;        // 距离大于它才展开（回差 15 cm，抑制临界抖动）
constexpr uint32_t FOLD_DEBOUNCE_MS = 300;      // 条件连续满足这么久才切状态
constexpr uint32_t FOLD_MOVE_TIMEOUT_MS = 2000; // 机构动作超时 → FAULT 并停机
constexpr float FOLD_STOW_RAD = 0.0f;           // 收折位置（相对上电位置，待实测）
constexpr float FOLD_DEPLOY_RAD = 1.2f;         // 展开位置（相对上电位置，待实测）

enum class FoldState : uint8_t
{
  IDLE = 0,      // 不动作（未使能 / 条件不满足）
  FOLDING = 1,   // 正在收折
  FOLDED = 2,    // 已收折
  UNFOLDING = 3, // 正在展开
  FAULT = 4      // 超时 / 异常，已停机，需要重新上电或复位
};

// 折叠决策（在 FoldTask 里周期调用，周期建议 10 ms）
void fold_update(uint32_t now_ms);

FoldState fold_state();

// 折叠电机的目标角（rad，相对上电位置）。
// 返回 false = 现在不该驱动折叠电机（未使能 / 未到动作条件 / 已到位 / FAULT）。
bool fold_target_rad(float * target_rad);

const char * fold_state_name(FoldState s);

#endif  // FOLD_TASK_HPP
