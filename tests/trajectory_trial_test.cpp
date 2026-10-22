#include "trajectory_trial.hpp"
#include "gimbal_task.hpp"
#include <cstdio>
#include <cstdlib>
#include <limits>

static void require(bool condition, const char * message) {
  if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}

int main() {
  using namespace trajectory_trial;
  constexpr float deg = 0.017453292519943295f;
  const auto inspect_at = [](float scale, float pitch, float yaw, float yr, float pr) {
    return inspect(scale, pitch, yaw, 0.03f, -0.349066f, 0.349066f, 0.785398f, yr, pr);
  };
  require(!inspect_at(0.15f, 0.0f, 0.0f, 1.5f, 1.5f).valid,
          "original 0.15 with lead slightly exceeds its old pitch slew cap");
  require(inspect_at(0.15f, 0.0f, 0.0f, 1.5f, 1.6f).valid,
          "0.15 fits when lead derivative is allowed");
  require(!inspect_at(0.5f, 0.0f, 0.0f, 4.5f, 5.5f).valid,
          "0.5 anchored at zero exceeds +20 degrees");
  const auto half = inspect_at(0.5f, -15.0f * deg, 0.0f, 4.5f, 5.5f);
  require(half.valid, "0.5 anchored at -15 degrees fits unchanged position bounds");
  require(half.yaw_peak_rate > 3.57f && half.yaw_peak_rate < 3.58f,
          "half yaw speed is about 3.574 rad/s");
  require(half.pitch_peak_rate > 4.0f && half.pitch_peak_rate < 5.5f,
          "half pitch lead requires substantially more than 1.5 rad/s");
  require(!inspect_at(0.5f, -15.0f * deg, 0.0f, 1.5f, 1.5f).valid,
          "old speed caps reject half trajectory");
  require(!inspect_at(1.0f, -30.0f * deg, 0.0f, 12.0f, 12.0f).valid,
          "full amplitude cannot fit forty-degree pitch window");
  require(!inspect_at(0.5f, -15.0f * deg, -40.0f * deg, 4.5f, 5.5f).valid,
          "yaw anchor near boundary is rejected");
  require(!inspect_at(0.0f, 0.0f, 0.0f, 4.5f, 5.5f).valid, "zero scale is invalid");
  require(!inspect_at(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f, 4.5f, 5.5f).valid,
          "NaN configuration is invalid");
  const auto relative = inspect(0.5f, 0.0f, 0.0f, 0.03f, -100.0f, 100.0f, 100.0f, 4.5f, 5.5f);
  const auto cascade_reference = inspect(0.5f, 0.0f, 0.0f, 0.0f,
                                          -100.0f, 100.0f, 100.0f, 4.5f, 5.5f);
  require(cascade_reference.valid && cascade_reference.pitch_peak_rate > 4.15f &&
          cascade_reference.pitch_peak_rate < 4.16f,
          "cascade reference without old lead needs 4.154 rad/s");
  require(relative.valid && fits(relative, -15.0f * deg, 0.0f, -20.0f * deg, 20.0f * deg, -45.0f * deg, 40.0f * deg),
          "cached envelope accepts valid entry");
  require(!fits(relative, 0.0f, 0.0f, -20.0f * deg, 20.0f * deg, -45.0f * deg, 40.0f * deg),
          "cached envelope rejects invalid entry");
  const Envelope point{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, true};
  require(!PITCH_POSITION_LIMIT_ENABLE, "production pitch angle limits disabled");
  require(std::fabs(pitch_limit_target(75.0f * deg) - 75.0f * deg) < 0.000001f,
          "positive target above former bound passes unchanged");
  require(std::fabs(pitch_limit_target(-75.0f * deg) + 75.0f * deg) < 0.000001f,
          "negative target below former bound passes unchanged");
  require(std::fabs(pitch_limit_target(75.0f * deg, true) - 50.0f * deg) < 0.000001f,
          "optional fallback clamp remains available");
  require(fits(relative, 40.0f * deg, 0.0f, PITCH_LIMIT_MIN_RAD, PITCH_LIMIT_MAX_RAD,
               YAW_SOFT_MIN_RAD, YAW_SOFT_MAX_RAD, PITCH_POSITION_LIMIT_ENABLE),
          "entry no longer rejected by pitch range");
  require(!fits(relative, 40.0f * deg, 45.0f * deg, PITCH_LIMIT_MIN_RAD, PITCH_LIMIT_MAX_RAD,
                YAW_SOFT_MIN_RAD, YAW_SOFT_MAX_RAD, PITCH_POSITION_LIMIT_ENABLE),
          "yaw range still rejects entry when pitch bounds disabled");
  require(!fits(relative, std::numeric_limits<float>::quiet_NaN(), 0.0f,
                PITCH_LIMIT_MIN_RAD, PITCH_LIMIT_MAX_RAD, YAW_SOFT_MIN_RAD,
                YAW_SOFT_MAX_RAD, PITCH_POSITION_LIMIT_ENABLE),
          "nonfinite pitch still rejected when angle bounds disabled");
  require(fits(relative, -2.76f * deg, 0.0f, PITCH_LIMIT_MIN_RAD, PITCH_LIMIT_MAX_RAD,
               YAW_SOFT_MIN_RAD, YAW_SOFT_MAX_RAD),
          "half trajectory fits current -2.76 degree pitch after range expansion");
  require(fits(relative, 4.07f * deg, 0.0f, PITCH_LIMIT_MIN_RAD, PITCH_LIMIT_MAX_RAD,
               YAW_SOFT_MIN_RAD, YAW_SOFT_MAX_RAD),
          "half trajectory also fits earlier +4.07 degree pitch");
  require(fits(relative, 5.0f * deg, 0.0f, PITCH_LIMIT_MIN_RAD, PITCH_LIMIT_MAX_RAD,
               YAW_SOFT_MIN_RAD, YAW_SOFT_MAX_RAD),
          "half trajectory from +5 degrees now fits");
  require(fits(relative, 19.0f * deg, 0.0f, PITCH_LIMIT_MIN_RAD, PITCH_LIMIT_MAX_RAD,
               YAW_SOFT_MIN_RAD, YAW_SOFT_MAX_RAD),
          "half trajectory from +19 degrees fits full envelope");
  require(!fits(relative, 20.0f * deg, 0.0f, PITCH_LIMIT_MIN_RAD, PITCH_LIMIT_MAX_RAD,
                YAW_SOFT_MIN_RAD, YAW_SOFT_MAX_RAD),
          "half trajectory from +20 degrees still exceeds +50 with lead");
  require(fits(point, 50.0f * deg, 0.0f, PITCH_LIMIT_MIN_RAD, PITCH_LIMIT_MAX_RAD,
               YAW_SOFT_MIN_RAD, YAW_SOFT_MAX_RAD), "+50 degree pitch boundary allowed");
  require(fits(point, -50.0f * deg, 0.0f, PITCH_LIMIT_MIN_RAD, PITCH_LIMIT_MAX_RAD,
               YAW_SOFT_MIN_RAD, YAW_SOFT_MAX_RAD), "-50 degree pitch boundary allowed");
  require(!fits(point, 50.01f * deg, 0.0f, PITCH_LIMIT_MIN_RAD, PITCH_LIMIT_MAX_RAD,
                YAW_SOFT_MIN_RAD, YAW_SOFT_MAX_RAD), "above +50 degree pitch rejected");
  require(!fits(point, -50.01f * deg, 0.0f, PITCH_LIMIT_MIN_RAD, PITCH_LIMIT_MAX_RAD,
                YAW_SOFT_MIN_RAD, YAW_SOFT_MAX_RAD), "below -50 degree pitch rejected");
  require(fits(point, 0.0f, 40.0f * deg, -1.0f, 1.0f, YAW_SOFT_MIN_RAD, YAW_SOFT_MAX_RAD),
          "+40 degree boundary allowed");
  require(!fits(point, 0.0f, 40.01f * deg, -1.0f, 1.0f, YAW_SOFT_MIN_RAD, YAW_SOFT_MAX_RAD),
          "above +40 degrees rejected");
  require(fits(point, 0.0f, -45.0f * deg, -1.0f, 1.0f, YAW_SOFT_MIN_RAD, YAW_SOFT_MAX_RAD),
          "negative -45 degree boundary retained");
  require(!fits(point, 0.0f, -45.01f * deg, -1.0f, 1.0f, YAW_SOFT_MIN_RAD, YAW_SOFT_MAX_RAD),
          "below -45 degrees rejected");
  require(std::fabs(yaw_error(-179.0f * deg, 179.0f * deg) - 2.0f * deg) < 0.00001f,
          "yaw crossing pi takes positive short path");
  require(std::fabs(yaw_error(179.0f * deg, -179.0f * deg) + 2.0f * deg) < 0.00001f,
          "yaw crossing pi takes negative short path");
  const auto start = trajectory_sample_at_ms(0);
  const auto wrap = trajectory_sample_at_ms(680);
  require(std::fabs(start.yaw - 0.314159265f) < 0.000001f &&
          std::fabs(start.pitch + 0.523598776f) < 0.000001f, "official start remains unchanged");
  require(wrap.yaw == start.yaw && wrap.pitch == start.pitch, "period stays 680 ms");
  std::printf("PASS: trajectory trial envelope, speed, period, yaw wrap\n");
  std::printf("0.5 pitch envelope at -15deg: %.3f .. %.3f deg; rates yaw %.3f pitch+lead %.3f rad/s\n",
              half.pitch_min / deg, half.pitch_max / deg, half.yaw_peak_rate, half.pitch_peak_rate);
}
