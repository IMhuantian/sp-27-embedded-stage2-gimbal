#pragma once
#include <cstdint>

struct ImuFeedback {
  float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
  uint32_t stamp_ms = 0;
  bool ready = false;
};

// Complete application-level snapshot; does not assert SPI transaction health.
ImuFeedback imu_feedback_read();
