/*
 * 4.4.7 云台自动折叠：决策状态机骨架
 *
 * 本轮只出框架（机构未装、折叠电机未接线），所以：
 *   - FOLD_ENABLE = false 时 fold_update() 直接把状态机压回 IDLE；
 *   - 动作完成的判据（位置到达 / 限位开关）与"限位异常退出"留成 TODO，接线后补。
 *
 * 输入：tof_task.cpp 的 tof_get_distance()（带新鲜度判断，见 tof_task.hpp）
 * 输出：fold_target_rad() 给 gimbal_task.cpp 的折叠电机位置环
 */

#include "fold_task.hpp"

#include "cmsis_os.h"

#include "tof_task.hpp"

namespace
{
FoldState state_ = FoldState::IDLE;

bool has_target_ = false;
float target_rad_ = 0.0f;

// 去抖：条件连续满足 FOLD_DEBOUNCE_MS 才允许切状态
bool cond_last_ = false;
uint32_t cond_since_ = 0;

// 动作超时（4.4.7 的"异常退出保护"）
uint32_t move_start_ = 0;
}  // namespace

FoldState fold_state()
{
  return state_;
}

bool fold_target_rad(float * target_rad)
{
  if (target_rad != nullptr) *target_rad = has_target_ ? target_rad_ : 0.0f;
  return has_target_;
}

const char * fold_state_name(FoldState s)
{
  switch (s) {
    case FoldState::IDLE:
      return "IDLE";
    case FoldState::FOLDING:
      return "FOLDING";
    case FoldState::FOLDED:
      return "FOLDED";
    case FoldState::UNFOLDING:
      return "UNFOLDING";
    case FoldState::FAULT:
      return "FAULT";
    default:
      return "?";
  }
}

void fold_update(uint32_t now_ms)
{
  if (!FOLD_ENABLE) {
    // 框架阶段：不决策、不驱动
    state_ = FoldState::IDLE;
    has_target_ = false;
    target_rad_ = 0.0f;
    cond_last_ = false;
    return;
  }

  // ---- 1) 取测距（带新鲜度：超过 TOF_TIMEOUT_MS 没更新视为无效）----
  float dist_cm = -1.0f;
  const bool dist_ok = tof_get_distance(&dist_cm, now_ms);

  // ---- 2) 条件 + 回差 + 去抖（避免阈值附近频繁切换）----
  //     收折条件：距离有效且 < 触发阈值；展开条件：距离有效且 > 释放阈值；
  //     距离无效（超时/帧错）时保持当前状态不动，不因为"看不见"就乱动机构。
  bool cond = false;
  if (dist_ok) {
    if (state_ == FoldState::FOLDED && !has_target_) cond = false;
    if (state_ == FoldState::FOLDED) {
      cond = (dist_cm > FOLD_RELEASE_CM);       // 已收折 → 只看释放阈值
    } else {
      cond = (dist_cm < FOLD_TRIGGER_CM);       // 其余状态 → 只看触发阈值
    }
  }
  if (cond != cond_last_) {
    cond_last_ = cond;
    cond_since_ = now_ms;
  }
  const bool cond_stable = cond && (now_ms - cond_since_ >= FOLD_DEBOUNCE_MS);

  // ---- 3) 状态机 ----
  switch (state_) {
    case FoldState::IDLE:
      if (cond_stable) {
        state_ = FoldState::FOLDING;
        has_target_ = true;
        target_rad_ = FOLD_STOW_RAD;
        move_start_ = now_ms;
      }
      break;

    case FoldState::FOLDING:
      // TODO(接线后)：到位判据 —— 位置误差 < 阈值 或 限位开关闭合；这里先按固定位置判断
      if (cond && (now_ms - move_start_ < FOLD_MOVE_TIMEOUT_MS)) {
        has_target_ = true;
        target_rad_ = FOLD_STOW_RAD;
      } else if (cond) {
        // 超时 → 异常退出保护：停机，等人工复位
        state_ = FoldState::FAULT;
        has_target_ = false;
        target_rad_ = 0.0f;
      } else {
        state_ = FoldState::FOLDED;
        has_target_ = false;
        target_rad_ = 0.0f;
      }
      break;

    case FoldState::FOLDED:
      if (cond_stable) {
        state_ = FoldState::UNFOLDING;
        has_target_ = true;
        target_rad_ = FOLD_DEPLOY_RAD;
        move_start_ = now_ms;
      }
      break;

    case FoldState::UNFOLDING:
      if (cond && (now_ms - move_start_ < FOLD_MOVE_TIMEOUT_MS)) {
        has_target_ = true;
        target_rad_ = FOLD_DEPLOY_RAD;
      } else if (cond) {
        state_ = FoldState::FAULT;
        has_target_ = false;
        target_rad_ = 0.0f;
      } else {
        state_ = FoldState::IDLE;
        has_target_ = false;
        target_rad_ = 0.0f;
      }
      break;

    case FoldState::FAULT:
    default:
      // 已停机：只有重新上电(或后续加一个显式复位接口)才会离开
      has_target_ = false;
      target_rad_ = 0.0f;
      break;
  }
}

// 4.4.7 的折叠决策任务：独立 FreeRTOS 任务（规则书要求），周期 10 ms
extern "C" void fold_task()
{
  while (true) {
    fold_update(osKernelSysTick());
    osDelay(10);
  }
}
