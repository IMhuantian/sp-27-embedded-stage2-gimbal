#include "traj_task.hpp"

#include "cmsis_os.h"

namespace {

volatile uint32_t s_start_ms = 0u;
volatile bool s_running = false;
volatile float s_yaw = 0.0f;
volatile float s_yaw_vel = 0.0f;
volatile float s_yaw_acc = 0.0f;
volatile float s_pitch = 0.0f;
volatile float s_pitch_vel = 0.0f;
volatile float s_pitch_acc = 0.0f;

}  // namespace

void traj_task_start(uint32_t start_ms)
{
  s_start_ms = start_ms;
  s_running = true;
}

void traj_task_stop() { s_running = false; }

TrajectoryPoint traj_task_latest()
{
  TrajectoryPoint out;
  out.yaw = s_yaw;
  out.yaw_vel = s_yaw_vel;
  out.yaw_acc = s_yaw_acc;
  out.pitch = s_pitch;
  out.pitch_vel = s_pitch_vel;
  out.pitch_acc = s_pitch_acc;
  return out;
}

// 4.4.8：独立任务（规则要求），1 kHz 采样组委会 681 点轨迹表
extern "C" void traj_task(void const * argument)
{
  (void)argument;

  for (;;) {
    if (s_running) {
      const TrajectoryPoint tp = trajectory_sample_at_ms(osKernelSysTick() - s_start_ms);
      s_yaw = tp.yaw;
      s_yaw_vel = tp.yaw_vel;
      s_yaw_acc = tp.yaw_acc;
      s_pitch = tp.pitch;
      s_pitch_vel = tp.pitch_vel;
      s_pitch_acc = tp.pitch_acc;
    }
    osDelay(1);
  }
}
