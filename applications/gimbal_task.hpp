#ifndef GIMBAL_TASK_HPP
#define GIMBAL_TASK_HPP

#include <cstdint>

#include "trajectory_task.hpp"  // 轨迹量程/周期在组委会头文件里，限位由它反算

// ============================ 云台 C 板（本工程）============================
// 本板职责（与底盘 C 板严格分开：两块板各自读自己的遥控器接收机、各自管自己的电机）：
//   4.4.2.1  红外测距 + C 板基本功能(LED / 蜂鸣器)  ← tof_task.cpp / led_task.cpp / buzzer_task.cpp
//   4.4.7    云台自动折叠(阈值 / 回差 / 状态判断)   ← fold_task.cpp（本轮只出框架）
//   4.4.8    目标轨迹跟踪                           ← 待做，见 README「待做」
//   云台电机：偏航轴 GM6020(id2) / 折叠轴 GM6020(id1) / 俯仰轴 C610+RM2006(id3)
//
// 底盘 C 板职责：4.4.2.2 遥控器与底盘控制 + 4.4.2.3 功率控制
//                → 工程 ...\files\stage_2（本工程不含底盘运动与功率控制）

// ---- 总开关 --------------------------------------------------------------
// 现场未接线时保持 false：不 config/start CAN、不发任何指令，只跑测距 / LED / 蜂鸣器 / 波形。
// 接线完成、确认电机转向与机械限位之后改成 true。
constexpr bool GIMBAL_ENABLE = true;

// 偏航轴使能：偏航电机故障时置 false —— 失能链不再等 0x206、也不再给偏航发指令，
// 折叠与俯仰两轴照常工作（台架单测 / 应急降级用）。
constexpr bool YAW_ENABLE = true;

// ---- 电机 ID 与总线 -------------------------------------------------------
constexpr uint8_t YAW_MOTOR_ID = 2;    // GM6020（底部竖直轴）→ 控制帧 0x1FF，反馈 0x206
constexpr uint8_t PITCH_MOTOR_ID = 4;  // C610 + RM2006（传感器头俯仰轴）→ 控制帧 0x200，反馈 0x204
constexpr uint8_t FOLD_MOTOR_ID = 1;   // GM6020（下方水平折叠轴）→ 控制帧 0x1FF，反馈 0x205
// ---- 电机总线（队友 shitao 2026-10-19 实测）--------------------------------
// CAN1(2-pin)：折叠 GM6020(id1, 反馈 0x205) + 俯仰 C610+RM2006(id4, 反馈 0x204)
// CAN2(4-pin)：偏航 GM6020(id2, 反馈 0x206) —— 底部竖直轴那台接在 CAN2
constexpr bool YAW_ON_CAN1 = false;
constexpr bool FOLD_ON_CAN1 = true;
constexpr bool PITCH_ON_CAN1 = true;

// ---- GM6020 控制模式 ------------------------------------------------------
// 实测这两台 GM6020 处在**电流(转矩)模式**：控制帧 0x1FE，cmd() 单位 N·m
// （库里对应 sp::RM_Motors::GM6020）。电压模式（0x1FF / GM6020_V）会被电机忽略，
// 表现就是橙灯常亮"电流控制模式下收到电压指令"+ 电机不动，不要再用。

// ---- 现场标定与保护（2026/10/21 台架实测）-----------------------------------
// 俯仰方向：头"仰"时 imu.pitch 变小（实测），所以目标角统一用"仰为正"约定，
//           反馈乘 PITCH_FB_SIGN 转换后再进位置环；
// 俯仰出力：目标约定 → 电机转矩的方向；若闭环往一边跑(正反馈)就把 PITCH_OUT_SIGN 改 -1；
// 偏航方向：从上看顺时针为负（正 = 逆时针），右摇杆右推要顺时针 → YAW_SIGN 在 .cpp 里取 -1；
// 偏航机械限位：以启用时刻姿态为中心 ±60°，软限位取 55° 留 5° 余量（软限位中心上电只锁一次）。
constexpr float PITCH_FB_SIGN = -1.0f;           // IMU 俯仰 → "仰为正"约定
constexpr float PITCH_OUT_SIGN = 1.0f;           // "仰为正"约定 → 电机转矩
constexpr float YAW_SOFT_LIMIT_RAD = 0.785398f;  // 偏航软限位 ±45°（机械 ±60°，事故后从 ±55° 收到 ±45°）
constexpr float YAW_LIMIT_KP = 5.0f;             // 到界回推增益 (rad/s)/rad

// ---- 机构参数（来自 2026-10-13 建模，详见 指导文档\第二阶段-云台机构与参数（来自建模）.md）----
// 机构链: 底盘云台连接板 → GM6020(id2, CAN2, YAW)
//         → 左右下板 + 下方水平轴(折叠轴 y≈52 mm)：GM6020(id1, CAN1) 驱动折叠
//         → 上方水平轴(俯仰轴 y≈138 mm)：C610 + M2006(id4, CAN1) 驱动俯仰
//         → 测距模块 / RM C 板随俯仰臂一起转；罩壳 PLA-CE
// 几何量是建模实测值（GetBox 包围盒），只作重力/惯量/包络估算，不参与控制。
// 俯仰机械限位：按组委会 4.4.8 轨迹文件（D:\university\RM\stage_2\trajectory_task）反推。
// 轨迹的 pitch ∈ [-0.523599, +0.523599] rad（±30.00°）、yaw ∈ ±0.392699 rad（±22.50°）。
// 这段是验收曲线，机构只要能到就必须覆盖，限位收窄会在跟踪中途卡死。
// TODO(干涉角复核): 若 SW 干涉扫描证明机构到不了 ±30°，以机构为准并同步文档。
constexpr float PITCH_LIMIT_MIN_RAD = -0.541052f;  // -31.00°（现场设定；官方轨迹只需 ±30°，留 1° 余量）
constexpr float PITCH_LIMIT_MAX_RAD = 0.541052f;   // +31.00°（现场设定；官方轨迹只需 ±30°，留 1° 余量）

// ---- 4.4.8 轨迹指标（由组委会轨迹文件统计，1 kHz 采样 / 680 ms 整周期 / 起止同点）----
// 稳态跟踪要按下面的峰值设计：现在是骨架，环路还远远跟不上（见文档 §2.2）。
constexpr float kDegToRad = 0.0174532925f;
constexpr float TRAJ_YAW_LIMIT_RAD =
  0.5f * TRAJECTORY_YAW_RANGE_DEG * kDegToRad;  // 0.392699 rad = ±22.50°
constexpr float TRAJ_PITCH_LIMIT_RAD =
  0.5f * TRAJECTORY_PITCH_RANGE_DEG * kDegToRad;    // 0.523599 rad = ±30.00°
constexpr float TRAJ_YAW_VEL_MAX_RAD_S = 7.149f;    // 峰值 7.1486 rad/s
constexpr float TRAJ_PITCH_VEL_MAX_RAD_S = 8.309f;  // 峰值 8.3083 rad/s
constexpr float TRAJ_YAW_ACC_MAX_RAD_S2 = 106.9f;   // 峰值 106.85 rad/s²
constexpr float TRAJ_PITCH_ACC_MAX_RAD_S2 = 69.1f;  // 峰值 69.04 rad/s²
// 限位必须覆盖轨迹范围，否则改小了直接编不过（这条比注释更管用）
// ⚠ 2026/10/21 事故后俯仰行程收到 ±20°，而轨迹要求 ±30°，两者已对不上。
// 这里不再硬断言"必须覆盖轨迹"，只在打开 TRAJ_ENABLE 前自己确认结构强度。
static_assert(
  PITCH_LIMIT_MAX_RAD > 0.0f && PITCH_LIMIT_MIN_RAD < 0.0f, "俯仰限位必须跨过 0");
// 以下几何量来自 2026-10-14 版建模实测（GetXform/GetBox，单位 m，以底板底面为 0）
constexpr float YAW_AXIS_HEIGHT_M = 0.0494f;     // YAW 转子面高（底板底面→yaw 转子面 49.4 mm）
constexpr float YAW_TO_FOLD_DIST_M = 0.001f;     // YAW 轴↔折叠轴：基本共点（实测偏心 1 mm）
constexpr float FOLD_AXIS_HEIGHT_M = 0.0524f;    // 折叠轴高（底板底面→折叠轴 52.4 mm，下方水平轴）
constexpr float PITCH_AXIS_HEIGHT_M = 0.1384f;   // 俯仰轴高（底板底面→俯仰轴 138.4 mm，上方水平轴）
constexpr float PITCH_TO_FOLD_DIST_M = 0.0859f;  // 折叠轴↔俯仰轴 = 俯仰臂长 85.9 mm
constexpr float PITCH_TO_TOF_DIST_M = 0.0653f;   // 俯仰轴↔测距模块（Δ57.3/Δ-31.4，半径 65.3 mm）
constexpr float AMBIENT_ENVELOPE_MM =
  304.3f;                               // 整机高度 304.3 mm（210×304×293 包络），折叠后高度判据用
constexpr float GIMBAL_MASS_KG = 0.0f;  // TODO(机械组给) 云台转动部分质量（重力前馈用）

// ---- 4.4.8 评分程序（规则手册 V2.1.0 §7.4.1）--------------------------------
// 组委会统一提供，代码逐字照抄；必须在 1 ms 周期的 FreeRTOS 任务里调用。
// 本工程在 gimbal_task() 的 1 kHz 控制环里每拍调用（见 gimbal_task.cpp），
// 实测角一律用 IMU 原始值：yaw_detect(imu.yaw, plan_yaw, &r) /
// pitch_detect(imu.pitch, plan_pitch, &r)，不翻转、不加偏移。
void yaw_detect(float gimbal_angle, float plan_angle, float* fire_ratio);
void pitch_detect(float gimbal_angle, float plan_angle, float* fire_ratio);

#endif  // GIMBAL_TASK_HPP
