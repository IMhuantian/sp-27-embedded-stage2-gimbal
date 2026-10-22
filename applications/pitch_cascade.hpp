#pragma once
#include <cmath>

namespace pitch_cascade {
struct Config {
  // Provisional bench values: 0.15 matches the old velocity damping coefficient;
  // 6 * 0.15 = 0.9 Nm/rad small-signal stiffness (old position Kp was 3).
  float position_kp = 6.0f;
  float speed_kp = 0.15f;
  float speed_ki = 0.3f;
  float max_rate = 5.5f;
  float max_torque = 0.3f;
  float torque_slew = 3.0f;
};
struct Input {
  bool enabled;
  float target_angle, actual_angle;
  float velocity_feedforward, measured_rate;
  float dt;
};
struct Output {
  float rate = 0.0f, raw_torque = 0.0f, torque = 0.0f;
  bool valid = true, rate_limited = false, torque_limited = false, slew_limited = false;
};
class Controller {
 public:
  Output step(const Config& config, const Input& input);
  void reset(float previous_torque = 0.0f);
 private:
  float integral_ = 0.0f, output_ = 0.0f;
};
}
