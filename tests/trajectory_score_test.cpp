#include "trajectory_score.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

static void check(bool ok, const char* message) {
  if (!ok) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}
static bool near(float value, float expected, float tolerance = 0.000002f) {
  return std::fabs(value - expected) < tolerance;
}
int main() {
  float yaw = 0.0f, pitch = 0.0f;
  yaw_detect(0.0f, 0.0f, nullptr);
  pitch_detect(0.0f, 0.0f, nullptr);
  yaw_detect(0.0f, 0.0f, &yaw);
  pitch_detect(0.0f, 0.0f, &pitch);
  check(near(yaw, 0.001f) && near(pitch, 0.001f), "official first perfect sample");
  yaw = pitch = 0.5f;
  yaw_detect(0.5f / 57.3f, 0.0f, &yaw);
  pitch_detect(1.5f / 57.3f, 0.0f, &pitch);
  check(near(yaw, 0.4995f) && near(pitch, 0.4995f), "strict threshold equality is a miss");
  yaw = pitch = 0.0f;
  yaw_detect(1.0f / 57.3f, 0.0f, &yaw);
  pitch_detect(1.0f / 57.3f, 0.0f, &pitch);
  check(yaw == 0.0f && near(pitch, 0.001f), "yaw and pitch use different thresholds");
  yaw = 0.0f;
  yaw_detect(3.14f, -3.14f, &yaw);
  check(yaw == 0.0f, "official scorer does not wrap yaw difference");

  using namespace trajectory_score;
  Session session;
  Sample s{100, true, true, false, 0.0f, 0.0f, 0.0f, 0.0f};
  auto r = session.observe(s);
  check(r.samples == 1 && near(r.mean_score, 0.04f), "one sample is 0.04 points, not forty");
  ++s.stamp_ms;
  r = session.observe(s);
  check(near(r.yaw_ratio, 0.001999f), "second official EMA sample");
  check(near(r.mean_score, 0.05998f), "score averages ratios across the run");
  check((r.flags & DebugAmplitude) != 0, "half-scale score is labelled debugging");
  s.active = false;
  r = session.observe(s);
  check(r.samples == 2 && !(r.flags & Running) && near(r.mean_score, 0.05998f),
        "stopping preserves the completed run");
  s.active = true; s.stamp_ms = 200; s.full_amplitude = true;
  r = session.observe(s);
  check(r.samples == 1 && near(r.mean_score, 0.04f) && !(r.flags & DebugAmplitude),
        "new run resets both EMA and mean");
  r = session.observe(s);
  check(r.samples == 1 && (r.flags & TimingInvalid), "same tick is not counted twice");
  s.stamp_ms += 3;
  r = session.observe(s);
  check(r.samples == 2 && (r.flags & TimingInvalid), "missing ticks are flagged, not fabricated");
  s.active = false; s.healthy = false;
  r = session.observe(s);
  check((r.flags & Aborted) != 0, "fault stop marks run aborted");
  s.active = true; s.healthy = true; s.stamp_ms = 0xffffffffu;
  session.observe(s); s.stamp_ms = 0;
  r = session.observe(s);
  check(r.samples == 2 && !(r.flags & TimingInvalid), "tick wrap keeps 1ms period");
  s.stamp_ms = 1; s.yaw_actual = std::numeric_limits<float>::quiet_NaN();
  r = session.observe(s);
  check(r.samples == 2 && (r.flags & FeedbackInvalid) && std::isfinite(r.mean_score),
        "nonfinite input cannot contaminate score");
  std::puts("PASS: official score thresholds, EMA, mean, reset, stop, timing and validity");
}
