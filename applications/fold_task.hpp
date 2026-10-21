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
constexpr float CHASSIS_SPEED_MPS = 0.35f;  // 只用于校核"0.8 m 够不够折完"（事故后车速按 0.35 m/s 算）
// 折叠动作时间：1.4 rad 行程（80°）、上限 1.2 rad/s、加减速 3 rad/s^2 的梯形规划 ≈ 1.4 s
constexpr float FOLD_MOVE_TIME_S = 2.2f;
constexpr uint32_t FOLD_MOVE_TIME_MS = 2200u;
// 触发余量：去抖 300 ms + 采样/刹停余量
constexpr float FOLD_TRIGGER_MARGIN_CM = 25.0f;
// 触发距离 = 车速 × 动作时间 + 余量，保证折叠在进隧道口之前做完（0.6×1.4×100+25 ≈ 109 cm）
// 触发距离按用户定稿固定 80 cm（原公式 车速×动作时间+余量 会算出 109 cm）。
// 校核见下面的 static_assert：0.8 m 在 CHASSIS_SPEED_MPS 下必须够折完。
constexpr float FOLD_TRIGGER_CM = 80.0f;
// 展开阈值：比触发阈值大（回差），又不能超过 GY-56 量程 120 cm
constexpr float FOLD_RELEASE_CM = 115.0f;
// 收折后至少保持这么久再允许展开（隧道长度 / 车速：3 s ≈ 1.8 m @0.6 m/s）
constexpr uint32_t FOLD_MIN_HOLD_MS = 3000u;
constexpr uint32_t FOLD_DEBOUNCE_MS = 300;       // 条件连续满足这么久才切状态
constexpr uint32_t FOLD_MOVE_TIMEOUT_MS = 3000;  // 动作超时 → FAULT 并停机
static_assert(
  FOLD_TRIGGER_CM < 120.0f, "触发距离超出 GY-56 量程 120cm：折叠来不及做完，加大折叠速度");
static_assert(FOLD_RELEASE_CM > FOLD_TRIGGER_CM, "释放阈值必须大于触发阈值（回差）");
static_assert(FOLD_TRIGGER_CM / (CHASSIS_SPEED_MPS * 100.0f) >= FOLD_MOVE_TIME_S,
              "80 cm 在 CHASSIS_SPEED_MPS 下不够折完：要么降车速，要么提高折叠速度/提前触发");
// ---- 折叠几何（2026/10/21 现场确认；2026/10/22 重新定义：上电 = 展开）----
// 上电姿态 = 折叠杆垂直底座 = **展开**位置，记为 0；折叠 = 从上电姿态反向转 80°。
// 方向只由 FOLD_STOW_DIR 决定（下面所有派生量都会跟着翻），反了就把它改成 +1.0f。
// 电机逆时针转时 fold_angle() 是变大还是变小取决于电机安装方向：
//   实测若"逆时针 → fold_angle() 变大"取 +1.0f；若"逆时针 → 变小"取 -1.0f。
//   只需改这一处，下面所有派生量（展开角、软限位）会自动跟着翻。
constexpr float FOLD_STOW_DIR = -1.0f;  // 折叠方向：-1 = 与展开相反（上电展开 → 往负方向折）；反了就改成 +1.0f
constexpr float FOLD_STOW_DEG = 80.0f;  // 折叠角（相对上电姿态，反向转 80°）
constexpr float FOLD_DEPLOY_RAD = 0.0f;  // 展开 = 上电姿态，记为 0
constexpr float FOLD_STOW_RAD =
  FOLD_STOW_DIR * FOLD_STOW_DEG * 0.017453292f;  // ≈ ∓1.396 rad
// 软限位：折叠方向留 5° 余量（85°），展开方向留 10° 余量（+10°）
constexpr float FOLD_LIMIT_STOW_RAD = FOLD_STOW_DIR * 1.483530f;   // 折叠方向 85°
constexpr float FOLD_LIMIT_DEPLOY_RAD = -FOLD_STOW_DIR * 0.174533f;  // 展开方向 10° 余量
constexpr float FOLD_LIMIT_LO_RAD =
  (FOLD_LIMIT_STOW_RAD < FOLD_LIMIT_DEPLOY_RAD) ? FOLD_LIMIT_STOW_RAD : FOLD_LIMIT_DEPLOY_RAD;
constexpr float FOLD_LIMIT_HI_RAD =
  (FOLD_LIMIT_STOW_RAD < FOLD_LIMIT_DEPLOY_RAD) ? FOLD_LIMIT_DEPLOY_RAD : FOLD_LIMIT_STOW_RAD;
static_assert(FOLD_STOW_RAD != FOLD_DEPLOY_RAD, "收折角与展开角相同，折叠不会动作");

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
