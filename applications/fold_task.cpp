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

// 折叠 + 复原只做一次：第一次收折开始就锁存，之后即使又测到 ≤80 cm 也不再折。
// 只有重新上电（静态变量重新初始化）才会放开；下档失能/切档都不重置。
bool cycle_done_ = false;
}  // namespace

bool fold_target_rad(float * target_rad)
{
  if (target_rad != nullptr) *target_rad = has_target_ ? target_rad_ : 0.0f;
  return has_target_;
}

bool fold_cycle_finished()
{
  // cycle_done_ 在第一次收折开始时置位；只有展开走完回到 IDLE 才算这一轮结束。
  return cycle_done_ && (state_ == FoldState::IDLE);
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

  // ---- 2) 收折条件 + 去抖 ----
  //     只有收折看测距：距离有效且 ≤ 触发阈值（读到 100 cm 就开始折）。
  //     复原不看测距：折完保持 FOLD_MIN_HOLD_MS 后自动展开（见状态机 FOLDED）。
  //     通信异常（无帧/超时/帧错）时不触发收折。
  const bool cond = dist_ok && dist_cm <= FOLD_TRIGGER_CM;
  if (cond != cond_last_) {
    cond_last_ = cond;
    cond_since_ = now_ms;
  }
  const bool cond_stable = cond && (now_ms - cond_since_ >= FOLD_DEBOUNCE_MS);
  const uint32_t move_elapsed = now_ms - move_start_;

  // ---- 3) 状态机（动作按“时间到”判定完成，不依赖距离怎么变）----
  switch (state_) {
    case FoldState::IDLE:
      // 只做一次：这一轮已经折过一次就不再触发（后面测到什么距离都不折）
      if (!cycle_done_ && cond_stable) {
        cycle_done_ = true;  // 收折一开始就锁存，避免折到一半回 IDLE 又折第二次
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
      // 折叠完成后保持 FOLD_MIN_HOLD_MS（3 s）自动复原，不再看测距
      if ((now_ms - folded_since_) >= FOLD_MIN_HOLD_MS) {
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
