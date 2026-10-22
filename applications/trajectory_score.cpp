#include "trajectory_score.hpp"
#include <cmath>

void yaw_detect(float gimbal_angle, float plan_angle, float* fire_ratio)
{
  if (fire_ratio == nullptr) return;
  constexpr float alpha = 0.001f;
  constexpr float threshold = 0.5f / 57.3f;
  static uint8_t fire = 0;
  fire = (std::fabs(gimbal_angle - plan_angle) < threshold) ? 1u : 0u;
  *fire_ratio = alpha * static_cast<float>(fire) + (1.0f - alpha) * (*fire_ratio);
}

void pitch_detect(float gimbal_angle, float plan_angle, float* fire_ratio)
{
  if (fire_ratio == nullptr) return;
  constexpr float alpha = 0.001f;
  constexpr float threshold = 1.5f / 57.3f;
  static uint8_t fire = 0;
  fire = (std::fabs(gimbal_angle - plan_angle) < threshold) ? 1u : 0u;
  *fire_ratio = alpha * static_cast<float>(fire) + (1.0f - alpha) * (*fire_ratio);
}

namespace trajectory_score {
Result latest;

const Result& Session::observe(const Sample& s)
{
  if (!s.active) {
    if (active_ && !s.healthy) result_.flags |= Aborted;
    active_ = false;
    result_.flags &= ~Running;
    return result_;
  }
  if (!active_) {
    result_ = {};
    yaw_sum_ = pitch_sum_ = 0.0;
    active_ = true;
  } else {
    const uint32_t elapsed = s.stamp_ms - last_ms_;
    if (elapsed != 1u) result_.flags |= TimingInvalid;
    if (elapsed == 0u) return result_;  // no manufactured catch-up samples
  }
  last_ms_ = s.stamp_ms;
  result_.flags |= Running;
  if (!s.full_amplitude) result_.flags |= DebugAmplitude;
  if (!s.healthy || !std::isfinite(s.yaw_actual) || !std::isfinite(s.yaw_target) ||
      !std::isfinite(s.pitch_actual) || !std::isfinite(s.pitch_target)) {
    result_.flags |= FeedbackInvalid;
    return result_;
  }
  yaw_detect(s.yaw_actual, s.yaw_target, &result_.yaw_ratio);
  pitch_detect(s.pitch_actual, s.pitch_target, &result_.pitch_ratio);
  ++result_.samples;
  result_.flags |= HasSamples;
  yaw_sum_ += result_.yaw_ratio;
  pitch_sum_ += result_.pitch_ratio;
  result_.yaw_mean = static_cast<float>(yaw_sum_ / result_.samples);
  result_.pitch_mean = static_cast<float>(pitch_sum_ / result_.samples);
  result_.mean_score = (result_.yaw_mean * 0.5f + result_.pitch_mean * 0.5f) * 40.0f;
  return result_;
}
}
