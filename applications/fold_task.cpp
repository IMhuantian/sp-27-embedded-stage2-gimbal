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
uint32_t folded_since_ = 0;  // 进入 FOLDED 的时刻（展开前的保持计时）
}  // namespace

bool fold_target_rad(float * target_rad)
{
  if (target_rad != nullptr) *target_rad = has_target_ ? target_rad_ : 0.0f;
  return has_target_;
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

  // ---- 0) 台架强制：不看测距，直接把目标角给定（测折叠电机/机构用）----
  if (FOLD_CMD != FoldCmd::AUTO) {
    const bool want_fold = (FOLD_CMD == FoldCmd::FOLD);
    state_ = want_fold ? FoldState::FOLDED : FoldState::IDLE;
    has_target_ = true;
    target_rad_ = want_fold ? FOLD_STOW_RAD : FOLD_DEPLOY_RAD;
    cond_last_ = false;
    return;
  }

  // ---- 1) 取测距（带新鲜度：超过 TOF_TIMEOUT_MS 没更新视为无效）----
  float dist_cm = -1.0f;
  const bool dist_ok = tof_get_distance(&dist_cm, now_ms);

  // ---- 2) 条件 + 回差 + 去抖（避免阈值附近频繁切换）----
  //     收折：距离有效且 ≤ 触发阈值（读到 80 cm 就开始折）；
  //     展开：距离 > 释放阈值（1 m），或“前方已经没有东西”（超量程/无效读数）；
  //     通信异常（无帧/超时/帧错）时保持当前状态，不因为"看不见"就乱动机构。
  const bool far = dist_ok ? (dist_cm > FOLD_RELEASE_CM) : (tof_status == TOF_STATUS_RANGE);
  const bool cond = (state_ == FoldState::FOLDED || state_ == FoldState::UNFOLDING)
                      ? far
                      : (dist_ok && dist_cm <= FOLD_TRIGGER_CM);
  if (cond != cond_last_) {
    cond_last_ = cond;
    cond_since_ = now_ms;
  }
  const bool cond_stable = cond && (now_ms - cond_since_ >= FOLD_DEBOUNCE_MS);
  const uint32_t move_elapsed = now_ms - move_start_;

  // ---- 3) 状态机（动作按“时间到”判定完成，不依赖距离怎么变）----
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
      // 目标角持续给 FOLD_STOW_RAD；走完 FOLD_MOVE_TIME_MS 才算到位（机构位置判据待实测补）
      has_target_ = true;
      target_rad_ = FOLD_STOW_RAD;
      if (move_elapsed >= FOLD_MOVE_TIME_MS) {
        state_ = FoldState::FOLDED;
        folded_since_ = now_ms;
      } else if (move_elapsed > FOLD_MOVE_TIMEOUT_MS) {
        state_ = FoldState::FAULT;  // 超时 → 异常退出保护，停机等人工复位
        has_target_ = false;
        target_rad_ = 0.0f;
      }
      break;

    case FoldState::FOLDED:
      // 折完先停稳 FOLD_MIN_HOLD_MS（0.5 s，避开收折规划最后一段），再看前方是否 > 1 m
      if (cond_stable && (now_ms - folded_since_ >= FOLD_MIN_HOLD_MS)) {
        state_ = FoldState::UNFOLDING;
        has_target_ = true;
        target_rad_ = FOLD_DEPLOY_RAD;
        move_start_ = now_ms;
      }
      break;

    case FoldState::UNFOLDING:
      has_target_ = true;
      target_rad_ = FOLD_DEPLOY_RAD;
      if (move_elapsed >= FOLD_MOVE_TIME_MS) {
        state_ = FoldState::IDLE;
        has_target_ = false;
        target_rad_ = 0.0f;
      } else if (move_elapsed > FOLD_MOVE_TIMEOUT_MS) {
        state_ = FoldState::FAULT;
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
