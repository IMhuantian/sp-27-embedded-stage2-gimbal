#pragma once

#include <cmath>
#include "trajectory_task.hpp"

namespace trajectory_trial {
constexpr float kPi = 3.14159265358979323846f;
inline float yaw_error(float target, float actual) {
  return std::remainder(target - actual, 2.0f * kPi);
}

struct Envelope {
  float pitch_min, pitch_max, yaw_min, yaw_max;
  float yaw_peak_rate, pitch_peak_rate;
  bool valid;
};

inline bool fits(const Envelope & envelope, float pitch_anchor, float yaw_anchor,
                 float pitch_lo, float pitch_hi, float yaw_lo, float yaw_hi,
                 bool pitch_limit_enabled = true, bool yaw_limit_enabled = true) {
  return std::isfinite(envelope.pitch_min) && std::isfinite(envelope.pitch_max) &&
    std::isfinite(envelope.yaw_min) && std::isfinite(envelope.yaw_max) &&
    std::isfinite(pitch_anchor) && std::isfinite(yaw_anchor) &&
    (!pitch_limit_enabled ||
      (envelope.pitch_min + pitch_anchor >= pitch_lo &&
       envelope.pitch_max + pitch_anchor <= pitch_hi)) &&
    (!yaw_limit_enabled ||
      (std::isfinite(yaw_lo) && std::isfinite(yaw_hi) && yaw_lo < yaw_hi &&
       envelope.yaw_min + yaw_anchor >= yaw_lo &&
       envelope.yaw_max + yaw_anchor <= yaw_hi));
}

// Checks the complete mapped reference, including the existing pitch lead.
// yaw_anchor is the motor angle relative to its power-on zero, not IMU yaw.
inline Envelope inspect(float amplitude, float pitch_anchor, float yaw_anchor,
                        float pitch_lead, float pitch_lo, float pitch_hi,
                        float yaw_limit, float yaw_rate_limit, float pitch_rate_limit) {
  Envelope out{INFINITY, -INFINITY, INFINITY, -INFINITY, 0.0f, 0.0f, false};
  if (!std::isfinite(amplitude) || amplitude <= 0.0f ||
      !std::isfinite(pitch_anchor) || !std::isfinite(yaw_anchor) ||
      !std::isfinite(pitch_lead) || !std::isfinite(pitch_lo) ||
      !std::isfinite(pitch_hi) || pitch_lo >= pitch_hi ||
      !std::isfinite(yaw_limit) || yaw_limit <= 0.0f ||
      !std::isfinite(yaw_rate_limit) || yaw_rate_limit <= 0.0f ||
      !std::isfinite(pitch_rate_limit) || pitch_rate_limit <= 0.0f) return out;
  const auto start = trajectory_sample_at_ms(0);
  for (uint32_t ms = 0; ms < TRAJECTORY_PERIOD_MS; ++ms) {
    const auto point = trajectory_sample_at_ms(ms);
    const float pitch = pitch_anchor + amplitude *
      (point.pitch - start.pitch + pitch_lead * point.pitch_vel);
    const float yaw = yaw_anchor + amplitude * (point.yaw - start.yaw);
    const float pr = std::fabs(amplitude * (point.pitch_vel + pitch_lead * point.pitch_acc));
    const float yr = std::fabs(amplitude * point.yaw_vel);
    if (!std::isfinite(pitch) || !std::isfinite(yaw) ||
        !std::isfinite(pr) || !std::isfinite(yr)) return out;
    out.pitch_min = std::fmin(out.pitch_min, pitch);
    out.pitch_max = std::fmax(out.pitch_max, pitch);
    out.yaw_min = std::fmin(out.yaw_min, yaw);
    out.yaw_max = std::fmax(out.yaw_max, yaw);
    out.pitch_peak_rate = std::fmax(out.pitch_peak_rate, pr);
    out.yaw_peak_rate = std::fmax(out.yaw_peak_rate, yr);
  }
  out.valid = out.pitch_min >= pitch_lo && out.pitch_max <= pitch_hi &&
    out.yaw_min >= -yaw_limit && out.yaw_max <= yaw_limit &&
    out.yaw_peak_rate <= yaw_rate_limit && out.pitch_peak_rate <= pitch_rate_limit;
  return out;
}
}  // namespace trajectory_trial
