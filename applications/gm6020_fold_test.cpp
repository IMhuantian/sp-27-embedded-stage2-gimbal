#include "gm6020_fold_test.hpp"

#include <cmath>

#include "cmsis_os.h"
#include "can.h"
#include "can_diag.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

#if FOLD_TEST_ENABLE

namespace
{
using namespace gm6020_fold_test;

constexpr float kRadToDeg = 57.2957795f;

static_assert(FOLD_TEST_ENABLE, "fold test source should only be used when enabled");
static_assert(CAN_DIAG_ENABLE, "fold test needs the dual-CAN receive path");
static_assert(kMotorOnCan1, "this test only supports the target GM6020 on CAN1");
static_assert(kMotorId == 1 && kExpectedRxId == 0x205, "target is CAN1 GM6020 ID1 / 0x205");
static_assert(kTravelRad > 0.0f, "travel must be positive");
static_assert(kMaxTorqueNm > 0.0f && kMaxTorqueNm < 0.741f * 3.0f, "torque limit out of range");

// 只用于发送：CAN 的 config/start 由 can_diag_start() 完成。
// 电流模式 GM6020：RM_Motors::GM6020 → tx 0x1FE（ID1~4），cmd 单位 N·m。
sp::RM_Motor motor(kMotorId, sp::RM_Motors::GM6020, 1.0f);
sp::PID pos_pid(1e-3f, kKp, 0.0f, kKd, kMaxTorqueNm, 0.0f, 1.0f);

volatile uint32_t last_feedback_ms = 0;

State state = State::WaitFeedback;
Fault fault = Fault::NoFeedback;

bool has_start = false;
bool settle_started = false;
bool auto_started = false;

float start_angle = 0.0f;
float target_out = 0.0f;
float target_back = 0.0f;
float ref_angle = 0.0f;
float rel_target = 0.0f;
float output = 0.0f;

uint32_t run_start_ms = 0;
uint32_t ready_since_ms = 0;
uint32_t settle_since_ms = 0;
uint32_t hold_start_ms = 0;
uint32_t last_loop_ms = 0;

bool config_valid()
{
  return kActionEnabled && (kDirection == +1 || kDirection == -1);
}

float clamp_output(float v)
{
  if (v > kMaxTorqueNm) return kMaxTorqueNm;
  if (v < -kMaxTorqueNm) return -kMaxTorqueNm;
  return v;
}

void output_zero()
{
  pos_pid.clear();
  output = 0.0f;
  motor.cmd(0.0f);
}

void abort(Fault reason)
{
  state = State::Aborted;
  fault = reason;
  output_zero();
}

void start_run(uint32_t now_ms)
{
  has_start = true;
  start_angle = motor.angle;
  target_out = start_angle + (float)kDirection * kTravelRad;
  target_back = start_angle;
  ref_angle = start_angle;
  rel_target = (float)kDirection * kTravelRad;
  run_start_ms = now_ms;
  settle_started = false;
  settle_since_ms = 0;
  hold_start_ms = 0;
  fault = Fault::None;
  state = State::MoveOut;
  auto_started = true;
  pos_pid.clear();
}

bool settled(uint32_t now_ms, float target, float angle, float speed)
{
  if (fabsf(target - angle) > kPosTolRad) {
    settle_started = false;
    return false;
  }
  if (fabsf(speed) > kSpeedTolRadS) {
    settle_started = false;
    return false;
  }
  if (!settle_started) {
    settle_started = true;
    settle_since_ms = now_ms;
    return false;
  }
  return (now_ms - settle_since_ms) >= kSettleMs;
}

void update_ref(float target, float dt)
{
  const float remain = target - ref_angle;
  float v = kMaxRefSpeedRadS;
  const float brake = sqrtf(2.0f * kMaxRefAccelRadS2 * fabsf(remain));
  if (brake < v) v = brake;
  const float step = v * dt;
  if (fabsf(remain) <= step) {
    ref_angle = target;
  }
  else {
    ref_angle += (remain > 0.0f) ? step : -step;
  }
}

void drive(float target, float dt)
{
  update_ref(target, dt);
  pos_pid.calc(ref_angle, motor.angle);
  output = clamp_output(pos_pid.out);
  if (!std::isfinite(output)) {
    abort(Fault::NonFinite);
    return;
  }
  motor.cmd(output);
}

bool send_std(CAN_HandleTypeDef * hcan, uint32_t tx_id, uint8_t * data)
{
  CAN_TxHeaderTypeDef header = {};
  header.StdId = tx_id;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8;

  uint32_t mailbox = 0;
  return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox) == HAL_OK;
}

bool send_frames(float torque_nm)
{
  // CAN1: 电流模式 0x1FE 第 1 槽 = ID1 转矩；其余槽全零。
  // 0x200 全零给 C610（M2006 电流模式），含义保持不变。
  if (can_diag::buses[0].start_ok) {
    uint8_t target[8] = {0};
    motor.cmd(torque_nm);
    motor.write(target);
    if (!send_std(&hcan1, motor.tx_id, target)) return false;

    uint8_t c610[8] = {0};
    if (!send_std(&hcan1, 0x200, c610)) return false;
  }
  else {
    motor.cmd(0.0f);
  }

  // CAN2: 电流模式 0x1FE 全零；绝不再发电压模式 0x1FF。
  if (can_diag::buses[1].start_ok) {
    uint8_t zero[8] = {0};
    if (!send_std(&hcan2, 0x1FE, zero)) return false;
  }

  return true;
}

void on_can_frame(const can_diag::Frame & frame)
{
  if (frame.bus != 0) return;
  if (!frame.standard || frame.remote || frame.dlc != 8) return;
  if (frame.id != kExpectedRxId) return;

  motor.read(frame.data, frame.now_ms);
  last_feedback_ms = frame.now_ms;
}

void fill_plot(float * plot, uint32_t now_ms)
{
  if (plot == nullptr) return;

  const float rel_actual = has_start ? (motor.angle - start_angle) : 0.0f;
  const float error = rel_target - rel_actual;
  const uint32_t feedback_age =
    (last_feedback_ms == 0) ? 0u : (now_ms - last_feedback_ms);

  plot[0] = (float)state;
  plot[1] = rel_target * kRadToDeg;
  plot[2] = rel_actual * kRadToDeg;
  plot[3] = error * kRadToDeg;
  plot[4] = motor.speed * kRadToDeg;
  plot[5] = output;
  plot[6] = (last_feedback_ms == 0) ? -1.0f : (float)feedback_age;
  plot[7] = auto_started ? 1.0f : 0.0f;
  plot[8] = (float)fault;
  plot[9] = (float)can_diag::online_mask(0, now_ms);
  plot[10] = (float)can_diag::online_mask(1, now_ms);
  plot[11] = (float)can_diag::buses[0].last_id;
  plot[12] = (float)can_diag::buses[1].last_id;
  plot[13] = (float)can_diag::buses[0].candidates[0].valid_count;
}
}  // namespace

namespace gm6020_fold_test
{
void gm6020_fold_test_loop(float * plot)
{
  can_diag::set_frame_hook(on_can_frame);
  const bool can_ok = can_diag::can_diag_start();

  if (!can_ok) {
    state = State::CanError;
    fault = Fault::CanError;
  }
  else if (!config_valid()) {
    state = State::ConfigLocked;
    fault = Fault::ConfigLocked;
  }
  else {
    state = State::WaitFeedback;
    fault = Fault::NoFeedback;
  }

  auto_started = false;
  ready_since_ms = 0;
  last_loop_ms = osKernelSysTick();

  while (true) {
    const uint32_t now_ms = osKernelSysTick();
    float dt = (float)(now_ms - last_loop_ms) * 0.001f;
    if (dt <= 0.0f || dt > 0.01f) dt = 0.001f;
    last_loop_ms = now_ms;

    const bool finite = std::isfinite(motor.angle) && std::isfinite(motor.speed);
    const bool fb_alive = motor.is_alive(now_ms) && finite;

    if (can_diag::state == 3u && state != State::CanError) {
      state = State::CanError;
      fault = Fault::CanError;
      output_zero();
    }

    if (state == State::CanError || !config_valid()) {
      output_zero();
      if (!send_frames(0.0f)) {
        state = State::CanError;
        fault = Fault::CanError;
      }
      fill_plot(plot, now_ms);
      osDelay(1);
      continue;
    }

    if (!finite) {
      if (state == State::MoveOut || state == State::Hold || state == State::MoveBack ||
          state == State::Ready) {
        abort(Fault::NonFinite);
      }
      else if (state == State::WaitFeedback) {
        fault = Fault::NonFinite;
      }
    }
    else if (!fb_alive) {
      if (state == State::MoveOut || state == State::Hold || state == State::MoveBack) {
        abort(Fault::FeedbackLost);
      }
      else if (state == State::Ready) {
        state = State::WaitFeedback;
        fault = Fault::NoFeedback;
        output_zero();
      }
      else if (state == State::WaitFeedback) {
        fault = Fault::NoFeedback;
      }
    }
    else if (state == State::WaitFeedback) {
      state = State::Ready;
      ready_since_ms = now_ms;
      fault = Fault::None;
    }

    // 无触发：首次有效反馈后等待 kAutoStartDelayMs，自动执行一次。
    if (state == State::Ready && (now_ms - ready_since_ms) >= kAutoStartDelayMs) {
      start_run(now_ms);
    }

    if (state == State::MoveOut || state == State::Hold || state == State::MoveBack) {
      if ((now_ms - run_start_ms) > kTimeoutMs) {
        abort(Fault::Timeout);
      }
      else {
        // 往/返/停留统一允许范围，覆盖 0° 起点和 +20° 端点：
        // [start - 0.15, start + 20° + 0.15]；越界仍中止。
        const float rel = motor.angle - start_angle;
        if (rel > (kTravelRad + kTravelMarginRad) || rel < -kTravelMarginRad) {
          abort(Fault::Travel);
        }
      }
    }

    switch (state) {
      case State::MoveOut:
        drive(target_out, dt);
        if (state == State::MoveOut && settled(now_ms, target_out, motor.angle, motor.speed)) {
          state = State::Hold;
          hold_start_ms = now_ms;
          settle_started = false;
          ref_angle = target_out;
        }
        break;

      case State::Hold:
        drive(target_out, dt);
        if (state == State::Hold && (now_ms - hold_start_ms) >= kHoldMs) {
          state = State::MoveBack;
          target_back = start_angle;
          rel_target = 0.0f;
          ref_angle = motor.angle;
          settle_started = false;
        }
        break;

      case State::MoveBack:
        drive(target_back, dt);
        if (state == State::MoveBack &&
            settled(now_ms, target_back, motor.angle, motor.speed)) {
          state = State::Done;
          output_zero();
        }
        break;

      case State::Done:
      case State::Aborted:
      case State::Ready:
      case State::WaitFeedback:
      case State::ConfigLocked:
      case State::CanError:
      default:
        output_zero();
        break;
    }

    if (!send_frames(output)) {
      state = State::CanError;
      fault = Fault::CanError;
      output_zero();
    }
    fill_plot(plot, now_ms);
    osDelay(1);
  }
}
}  // namespace gm6020_fold_test

#endif  // FOLD_TEST_ENABLE
