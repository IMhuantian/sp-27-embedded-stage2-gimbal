#include "pitch_cascade.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

static void check(bool ok, const char* text) {
  if (!ok) { std::fprintf(stderr, "FAIL: %s\n", text); std::exit(1); }
}
static bool near(float value, float expected) { return std::fabs(value - expected) < 0.000001f; }
int main() {
  using namespace pitch_cascade;
  Config c; c.position_kp = 2; c.speed_kp = 0.1f; c.speed_ki = 0;
  c.max_rate = 1; c.max_torque = 0.3f; c.torque_slew = 1000;
  Controller controller;
  Input i{true, 0.1f, 0.0f, 0.3f, 0.2f, 0.001f};
  auto out = controller.step(c, i);
  check(out.valid && near(out.rate, 0.5f) && near(out.torque, 0.03f),
        "position correction plus velocity feedforward enters measured speed loop");
  i.target_angle = i.actual_angle = 0; i.velocity_feedforward = 0; i.measured_rate = 0.2f;
  out = controller.step(c, i);
  check(near(out.torque, -0.02f), "positive measured rate is opposed by negative torque");
  i.target_angle = i.actual_angle = 0; i.velocity_feedforward = i.measured_rate = 0.3f;
  out = controller.step(c, i);
  check(near(out.torque, 0), "ideal rate tracking needs no proportional torque");
  i.target_angle = 10; i.measured_rate = i.velocity_feedforward = 0;
  out = controller.step(c, i);
  check(out.rate_limited && near(out.rate, 1) && near(out.torque, 0.1f),
        "outer loop speed limited separately from torque");
  c.speed_kp = 1; c.speed_ki = 1;
  for (int n = 0; n < 10000; ++n) out = controller.step(c, i);
  check(out.torque_limited && near(out.torque, 0.3f), "long torque saturation stays bounded");
  i.target_angle = 0;
  out = controller.step(c, i);
  check(near(out.torque, 0), "torque saturation does not leave a wound-up integral");
  c.max_torque = 10; c.torque_slew = 0.1f;
  i.target_angle = 10; controller.reset();
  out = controller.step(c, i);
  check(out.slew_limited && near(out.torque, 0.0001f), "first torque step respects slew");
  for (int n = 1; n < 100; ++n) out = controller.step(c, i);
  i.target_angle = 0;
  for (int n = 0; n < 200; ++n) out = controller.step(c, i);
  check(near(out.torque, 0), "slew-limited operation also prevents windup");
  c.torque_slew = 1000; c.speed_kp = 0; c.speed_ki = 1; i.target_angle = 0.1f;
  controller.reset(); out = controller.step(c, i);
  check(near(out.torque, 0.0002f), "unsaturated velocity integral accumulates in real seconds");
  controller.reset(); i.dt = 0.002f; out = controller.step(c, i);
  check(near(out.torque, 0.0004f), "2ms integral uses measured elapsed time");
  i.dt = 0.001f;
  i.enabled = false; out = controller.step(c, i);
  check(near(out.torque, 0), "disable immediately zeros output");
  i.enabled = true; i.target_angle = 0; out = controller.step(c, i);
  check(near(out.torque, 0), "re-enable has no old integral");
  i.measured_rate = std::numeric_limits<float>::quiet_NaN();
  out = controller.step(c, i);
  check(!out.valid && near(out.torque, 0), "nonfinite speed fails closed");
  i.measured_rate = 0; i.dt = 0;
  check(!controller.step(c, i).valid, "zero dt rejected");
  i.dt = 0.021f;
  check(!controller.step(c, i).valid, "long scheduling gap rejected");
  i.dt = 0.001f; c.speed_kp = -1;
  check(!controller.step(c, i).valid, "invalid negative gain rejected");
  c.speed_kp = 1; c.speed_ki = 0; c.max_torque = 0.3f; c.torque_slew = 3;
  controller.reset(0.2f); i.target_angle = 0;
  out = controller.step(c, i);
  check(near(out.torque, 0.197f), "mode takeover starts from actual previous output with slew");
  std::puts("PASS: pitch cascade public behavior, feedforward, limits, antiwindup, disable, validity");
}
