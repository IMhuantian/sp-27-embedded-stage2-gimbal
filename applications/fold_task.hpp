#ifndef FOLD_TASK_HPP
#define FOLD_TASK_HPP

#include <cstdint>

// ============================ 4.4.7 云台自动折叠（框架）============================
// 规则书要求：
//   - 用独立的 FreeRTOS 任务完成折叠决策与动作；
//   - 用测距结果 + 阈值/回差 + 状态判断，避免在阈值附近频繁切换；
//   - 有限位 / 超时 / 异常退出保护。
//
// 当前状态（2026-10-24）：机构与折叠电机已接线，FOLD_ENABLE = true：
//   收折：距离读到 ≤ FOLD_TRIGGER_CM(100 cm)；
//   复原：折叠完成后保持 FOLD_MIN_HOLD_MS(3 s) 自动展开，**不看测距**。
//   折叠 + 复原只做一次：第一次收折触发即锁存，之后无论测距如何都不再折；重新上电才放开。
//   动作到位仍按时间判（见 fold_task.cpp），限位开关未接。

constexpr bool FOLD_ENABLE = true;

// 阈值：按现场实测整定（GY-56 固有回差 ≥ 3 cm 量级）；只用于收折触发，复原不看距离
// ---- 时序（按底盘自动行驶速度推算；改车速或折叠速度时这几个数要一起改）----
// 底盘自动行驶速度（can_task.cpp 的 AUTO_SPEED_MPS），云台这边只能按设计值推算
constexpr float CHASSIS_SPEED_MPS = 0.55f;  // 只用于校核"1 m 够不够折完"（与底盘 AUTO_SPEED_MPS 对齐：0.42→0.55）
// 折叠动作时间：1.4 rad 行程（80°）、上限 1.5 rad/s、加减速 3.0 rad/s^2 的梯形规划 ≈ 1.4 s
constexpr float FOLD_MOVE_TIME_S = 1.5f;
constexpr uint32_t FOLD_MOVE_TIME_MS = 1500u;
// 触发余量：去抖 300 ms + 采样/刹停余量
constexpr float FOLD_TRIGGER_MARGIN_CM = 25.0f;
// 触发距离 = 车速 × 动作时间 + 余量，保证折叠在进隧道口之前做完（0.6×1.4×100+25 ≈ 109 cm）
// 触发距离按用户定稿固定 100 cm（2026-10-24 在 80 cm 基础上 +20 cm）；
// 判据是 "≤ 100 cm"：GY-56 读数是整数 cm，用 "<" 会漏掉正好读到 100 的那一格。
// 校核见下面的 static_assert：1 m 在 CHASSIS_SPEED_MPS 下必须够折完。
constexpr float FOLD_TRIGGER_CM = 100.0f;
// 折叠完成后保持 FOLD_MIN_HOLD_MS 就自动复原，不再看测距（原 120 cm 复原阈值已删除）
constexpr uint32_t FOLD_MIN_HOLD_MS = 3000u;
constexpr uint32_t FOLD_DEBOUNCE_MS = 300;       // 条件连续满足这么久才切状态
constexpr uint32_t FOLD_MOVE_TIMEOUT_MS = 3000;  // 动作超时 → FAULT 并停机
static_assert(
  FOLD_TRIGGER_CM < 120.0f, "触发距离超出 GY-56 量程 120cm：折叠来不及做完，加大折叠速度");
static_assert(FOLD_TRIGGER_CM / (CHASSIS_SPEED_MPS * 100.0f) >= FOLD_MOVE_TIME_S,
              "100 cm 在 CHASSIS_SPEED_MPS 下不够折完：要么降车速，要么提高折叠速度/提前触发");
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

// 折叠 + 复原这一轮是否已经走完（已回到展开位）。
// 上档自动轨迹用它当 20 s 倒计时的起点；没折过（或还在折/还没复原）返回 false。
bool fold_cycle_finished();

#endif  // FOLD_TASK_HPP
