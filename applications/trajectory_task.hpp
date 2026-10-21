#ifndef TRAJECTORY_TASK_HPP
#define TRAJECTORY_TASK_HPP

#include <stdint.h>

// 单位：角度 rad、速度 rad/s、加速度 rad/s^2。

#define TRAJECTORY_YAW_RANGE_DEG 45.0f
#define TRAJECTORY_PITCH_RANGE_DEG 60.0f

#define TRAJECTORY_SAMPLE_INTERVAL_MS 1U
#define TRAJECTORY_PERIOD_MS 680U
#define TRAJECTORY_YAW_MAX_ACCELERATION_RAD_S2 110.0f
#define TRAJECTORY_PITCH_MAX_ACCELERATION_RAD_S2 70.0f
#define TRAJECTORY_UPDATE_INTERVAL_MS 1U

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
} TrajectoryPoint;

TrajectoryPoint trajectory_sample_at_ms(uint32_t elapsed_ms);

#ifdef __cplusplus
}
#endif

#endif  // TRAJECTORY_TASK_HPP
