#include "pitch_cascade.hpp"

namespace pitch_cascade {
namespace {
float clamp(float value, float lo, float hi) {
  return value < lo ? lo : (value > hi ? hi : value);
}
bool finite(float value) { return std::isfinite(value); }
}
void Controller::reset(float previous_torque) {
  integral_ = 0.0f;
  output_ = finite(previous_torque) ? previous_torque : 0.0f;
}
Output Controller::step(const Config& c, const Input& i) {
  Output out;
  if (!i.enabled) { reset(); return out; }
  if (!finite(c.position_kp) || c.position_kp < 0.0f ||
      !finite(c.speed_kp) || c.speed_kp < 0.0f || !finite(c.speed_ki) || c.speed_ki < 0.0f ||
      !finite(c.max_rate) || c.max_rate <= 0.0f ||
      !finite(c.max_torque) || c.max_torque <= 0.0f ||
      !finite(c.torque_slew) || c.torque_slew <= 0.0f ||
      !finite(i.target_angle) || !finite(i.actual_angle) ||
      !finite(i.velocity_feedforward) || !finite(i.measured_rate) ||
      !finite(i.torque_feedforward) ||
      !finite(i.dt) || i.dt <= 0.0f || i.dt > 0.02f) {
    reset(); out.valid = false; return out;
  }
  const float requested_rate = c.position_kp * (i.target_angle - i.actual_angle) +
    i.velocity_feedforward;
  if (!finite(requested_rate)) { reset(); out.valid = false; return out; }
  out.rate = clamp(requested_rate, -c.max_rate, c.max_rate);
  out.rate_limited = out.rate != requested_rate;
  const float error = out.rate - i.measured_rate;
  const float proportional = c.speed_kp * error;
  const float candidate = clamp(integral_ + c.speed_ki * error * i.dt,
                                -c.max_torque, c.max_torque);
  // Feedforward torque (inertia compensation) is added here, inside the actuator
  // limits, so it can never bypass the torque cap or the output slew rate.
  const float raw = proportional + candidate + i.torque_feedforward;
  if (!finite(error) || !finite(proportional) || !finite(raw)) {
    reset(); out.valid = false; return out;
  }
  // Both stages constrain the actuator: do not integrate further into either.
  const float max_step = c.torque_slew * i.dt;
  output_ = clamp(output_, -c.max_torque, c.max_torque);
  const float proposed = clamp(clamp(raw, -c.max_torque, c.max_torque),
                               output_ - max_step, output_ + max_step);
  if ((raw - proposed) * error <= 0.0f) integral_ = candidate;
  out.raw_torque = proportional + integral_ + i.torque_feedforward;
  const float limited = clamp(out.raw_torque, -c.max_torque, c.max_torque);
  out.torque_limited = limited != out.raw_torque;
  out.torque = clamp(limited, output_ - max_step, output_ + max_step);
  out.slew_limited = out.torque != limited;
  output_ = out.torque;
  return out;
}
}
