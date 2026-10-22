#pragma once
#include <cstdint>

// Official functions supplied by the user from rules section 7.4.1.
void yaw_detect(float gimbal_angle, float plan_angle, float* fire_ratio);
void pitch_detect(float gimbal_angle, float plan_angle, float* fire_ratio);

namespace trajectory_score {
enum Flag : uint32_t {
  Running = 1u, HasSamples = 2u, TimingInvalid = 4u,
  FeedbackInvalid = 8u, DebugAmplitude = 16u, Aborted = 32u
};
struct Result {
  float yaw_ratio = 0.0f, pitch_ratio = 0.0f;
  float yaw_mean = 0.0f, pitch_mean = 0.0f;
  float mean_score = 0.0f;
  uint32_t samples = 0, flags = 0;
};
struct Sample {
  uint32_t stamp_ms;
  bool active, healthy, full_amplitude;
  float yaw_actual, yaw_target, pitch_actual, pitch_target;
};
class Session {
 public:
  const Result& observe(const Sample& sample);
 private:
  Result result_;
  double yaw_sum_ = 0.0, pitch_sum_ = 0.0;
  uint32_t last_ms_ = 0;
  bool active_ = false;
};
// Debugger-readable snapshot, reset by reboot/new trajectory run.
extern Result latest;
}
