#ifndef GIMBAL_TASK_HPP
#define GIMBAL_TASK_HPP

#include <cstdint>

// 云台侧已不再使用 DT7/遥控器；正常 yaw/pitch 遥控控制已移除。
// 当前保留：
//   - 双 CAN 接收诊断（can_diag）
//   - 单电机 20° 相对往返测试（gm6020_fold_test，板载 KEY 触发）
// 姿态接口保留给后续轨迹/折叠使用。
float gimbal_yaw_rad();
float gimbal_pitch_rad();

#endif  // GIMBAL_TASK_HPP
