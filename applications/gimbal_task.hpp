#ifndef GIMBAL_TASK_HPP
#define GIMBAL_TASK_HPP

#include <cstdint>

// ============================ 云台 C 板（本工程）============================
// 本板职责（与底盘 C 板严格分开：两块板各自读自己的遥控器接收机、各自管自己的电机）：
//   4.4.2.1  红外测距 + C 板基本功能(LED / 蜂鸣器)  ← tof_task.cpp / led_task.cpp / buzzer_task.cpp
//   4.4.7    云台自动折叠(阈值 / 回差 / 状态判断)   ← fold_task.cpp（本轮只出框架）
//   4.4.8    目标轨迹跟踪                           ← 待做，见 README「待做」
//   云台电机：偏航轴 GM6020(id1) / 俯仰轴 GM6020(id2) / 折叠机构 C610+RM2006(id3)
//
// 底盘 C 板职责：4.4.2.2 遥控器与底盘控制 + 4.4.2.3 功率控制
//                → 工程 ...\files\stage_2（本工程不含底盘运动与功率控制）

// ---- 总开关 --------------------------------------------------------------
// 现场未接线时保持 false：不 config/start CAN、不发任何指令，只跑测距 / LED / 蜂鸣器 / 波形。
// 接线完成、确认电机转向与机械限位之后改成 true。
constexpr bool GIMBAL_ENABLE = false;

// ---- 电机 ID 与总线 -------------------------------------------------------
constexpr uint8_t YAW_MOTOR_ID = 1;    // GM6020 → 控制帧 0x1FF，反馈 0x205
constexpr uint8_t PITCH_MOTOR_ID = 2;  // GM6020 → 控制帧 0x1FF，反馈 0x206
constexpr uint8_t FOLD_MOTOR_ID = 3;   // C610 + RM2006 → 控制帧 0x200，反馈 0x203
constexpr bool MOTORS_ON_CAN1 = true;  // 云台电机挂 CAN1（2-pin 口）

// ---- 给 4.4.7 折叠 / 4.4.8 轨迹跟踪用的姿态接口 ---------------------------
float gimbal_yaw_rad();    // IMU 偏航角 rad
float gimbal_pitch_rad();  // IMU 俯仰角 rad

#endif  // GIMBAL_TASK_HPP
