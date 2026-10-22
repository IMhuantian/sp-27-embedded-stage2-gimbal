#pragma once

#include <stdint.h>

#include "trajectory_task.hpp"

// 4.4.8 轨迹给定：独立 FreeRTOS 任务按 1 ms 采样组委会轨迹表，
// 云台控制环只读它发布的最新给定 —— 生成与控制解耦，控制环被抢占也不会错相位。
void traj_task_start(uint32_t start_ms);  // 从 start_ms 起跑（相位 0 落在 start_ms）
void traj_task_stop();                    // 停跑：发布值保持最后一拍
TrajectoryPoint traj_task_latest();       // 读最新给定（每 1 ms 更新一次）
