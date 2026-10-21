/*
 * 第二阶段 云台 C 板：云台电机(偏航 / 俯仰 / 折叠机构)控制
 *
 * 硬件：
 *   偏航轴 GM6020(id2，CAN2) / 折叠轴 GM6020(id1，CAN1) —— 电流模式，控制帧 0x1FE，反馈 0x206 / 0x205
 *   俯仰轴 C610 + RM2006(id3)               —— 转矩模式，         控制帧 0x200，反馈 0x203
 *   两条总线：CAN1 = 折叠 GM6020 + 俯仰 C610；CAN2 = 偏航 GM6020；IMU 走 SPI1(bmi088 + mahony，见 imu_task.cpp)；遥控器 DT7 走 USART3(见 uart_task.cpp)
 *
 * 遥控映射(云台侧；本板自己读 DT7，与底盘 C 板互不转发)：
 *   右拨杆下档 → 云台全部电机失能(最高优先级)
 *   右摇杆左右 → 偏航角速度给定(满推 YAW_RATE_MAX rad/s)
 *   左摇杆上下 → 俯仰角速度给定(增量式位置环：松手停在当前角，因为俯仰轴有重力负载)
 *   左摇杆左右 / 拨轮 → 暂未分配
 *   4.4.8 目标轨迹跟踪 → TRAJ_ENABLE=true 且右拨杆上档时接管偏航/俯仰给定（组委会 trajectory_task）
 *
 * 安全：
 *   1) 三层失能：拨杆下档 / 遥控器失联 / 电机反馈中断 —— 每周期把三个电机都发 0
 *   2) 全部输出限幅(PID 总输出 + 积分分项 + 电压 / 转矩上限)
 *   3) GIMBAL_ENABLE=false 时根本不 config/start CAN，台架上可以安全地只调测距
 */

#include "gimbal_task.hpp"

#include <cmath>

#include "cmsis_os.h"
#include "fold_task.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tof_task.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"
#include "trajectory_task.hpp"

// 定义在 uart_task.cpp / imu_task.cpp
extern sp::DBus remote;
extern sp::Mahony imu;

// SerialPlot 通道(定义在这里，plotter_task.cpp 里 extern)
//   0  测距(3 点中值) cm        1  测距状态码          2/3/4  IMU yaw / pitch / roll rad
//   5  偏航目标角速度          6  偏航实际角速度      7  偏航控制量 V
//   8  俯仰目标角(仰为正)      9  俯仰实际角(仰为正)   10 俯仰控制量 N·m
//   11 失能原因码(调试)        12 反馈帧位图(调试)    13 右开关档位(调试: 0 下 / 1 中 / 2 上)
float plot_ch[14] = {0.0f};

// 失能原因码（调试）：0 正常 / 1 拨杆下档 / 2 遥控失联 / 3 偏航反馈丢 / 4 俯仰反馈丢
uint8_t gimbal_fault_ = 0;
// 收到过的反馈帧位图（调试）：bit(n) = 收到过 0x203+n
//   bit0 0x203 = C610 id3   bit1 0x204 = C610 id4(俯仰应在此)
//   bit2 0x205 = GM6020 id1(折叠应在此)   bit3 0x206 = GM6020 id2(偏航应在此)
//   bit4 0x207 = GM6020 id3   bit5 0x208 = GM6020 id4
uint16_t gimbal_rx_seen_ = 0;
uint32_t can1_rx_ms_ = 0;  // CAN1 最近一次收到电机反馈（0 = 从未收到）
uint32_t can2_rx_ms_ = 0;  // CAN2 同上

namespace
{
constexpr float CONTROL_DT = 1e-3f;  // 控制周期 1 ms

// ---- 给定与限幅 ----------------------------------------------------------
constexpr float YAW_RATE_MAX = 6.0f;    // rad/s：右摇杆左右满推
constexpr float PITCH_RATE_MAX = 1.5f;  // rad/s：左摇杆上下满推
// 俯仰机械限位的唯一出处是 gimbal_task.hpp 里的 PITCH_LIMIT_MIN/MAX_RAD（值待建模实测）
constexpr float PITCH_MIN_RAD = PITCH_LIMIT_MIN_RAD;  // 俯仰机械下限
constexpr float PITCH_MAX_RAD = PITCH_LIMIT_MAX_RAD;  // 俯仰机械上限
constexpr float YAW_MAX_TORQUE = 0.5f;    // GM6020 电流模式满量程 2.223 N·m；先给 0.5，整定时再放开
constexpr float PITCH_MAX_TORQUE = 1.0f;  // RM2006 额定转矩约 1 N·m（俯仰是转矩模式）
constexpr float FOLD_MAX_TORQUE = 0.5f;   // 折叠 GM6020 电流模式，先给 0.5 N·m
constexpr float STICK_DEADZONE = 0.02f;

// 方向：现场确认后如反了就把对应 SIGN 取反（与底盘工程 FWD_DIR / SIDE_DIR / ROT_DIR 同一套路）
constexpr float YAW_SIGN = -1.0f;  // 从上看顺时针为负；右推(顺时针) → 负速率
constexpr float PITCH_SIGN = 1.0f;
constexpr float YAW_OUT_SIGN = 1.0f;  // 偏航转矩方向；闭环若往一边跑就改成 -1

constexpr uint32_t STARTUP_WAIT_MS = 1000;  // 等电机上电自检 + 第一帧反馈

// ---- 4.4.8 目标轨迹跟踪（组委会 trajectory_task，默认关）------------------
// 轨迹指标：680 ms 整周期、1 kHz 采样、起止同点；pitch ±30.00°（峰值 8.31 rad/s / 69 rad/s²），
// yaw ±22.50°（峰值 7.15 rad/s / 107 rad/s²）。
// 实现选择：直接调 trajectory_sample_at_ms() 取给定，不再单开一个任务 —— 控制环本身就是
// 1 kHz，多一个任务只会多一层调度抖动（组委会 readme 里那个任务照样编译进来，不用它也行）。
// 跟踪策略：前馈 + 外环，而不是纯 PI 追误差。
//   yaw  : 角度外环 P → 角速度给定 = Kp*(θ_ref − θ_imu) + k_v*ω_ref → 原有速率环
//   pitch: 参考角超前 lead 秒（等效 ω/a 前馈）→ 位置环；再叠一个加速度转矩前馈
// 两个前馈增益都从安全值起步，不接线时对输出没有任何影响。
constexpr bool TRAJ_ENABLE = true;  // 编译期总开关；运行时还要把右拨杆打到上档才接管（中档仍是手动）
constexpr float TRAJ_YAW_KP = 6.0f;         // 1/s        角度误差 → 角速度给定
constexpr float TRAJ_YAW_VEL_FF = 1.0f;     // 轨迹角速度前馈系数（1.0 = 直接叠加）
constexpr float TRAJ_YAW_RATE_LIM = 10.0f;  // rad/s      跟踪时角速度上限（轨迹峰值 7.15）
constexpr float TRAJ_PITCH_LEAD_S = 0.03f;  // s          参考角超前量（等效速度前馈）
constexpr float TRAJ_PITCH_ACC_FF = 0.0f;   // N·m/(rad/s²) 加速度转矩前馈（标定项，默认 0）

// ---- 电机与总线 ----------------------------------------------------------
// 两条总线：偏航在 CAN2，折叠与俯仰在 CAN1（队友 2026-10-19 实测接线）
sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);
sp::RM_Motor yaw_motor(YAW_MOTOR_ID, sp::RM_Motors::GM6020, 1.0f);  // 电流模式：0x1FE，cmd 单位 N·m
// ⚠️ 俯仰轴用 M2006 自带减速箱(36:1)，机构上若还有一级减速要改这里
sp::RM_Motor pitch_motor(PITCH_MOTOR_ID, sp::RM_Motors::M2006, sp::M2006_P36);
sp::RM_Motor fold_motor(
  FOLD_MOTOR_ID, sp::RM_Motors::GM6020, 1.0f);  // 电流模式：0x1FE，cmd 单位 N·m

// ---- 控制器（输出都是"电压 V" / "转矩 N·m"，由 RM_Motor::cmd() 换算成原始值）----
sp::PID yaw_pid(CONTROL_DT, 0.6f, 0.02f, 0.0f, YAW_MAX_TORQUE, 6.0f, 0.7f);
// 俯仰(M2006 转矩模式)：Kp 1.5 N·m/rad，Ki 用来扛重力
sp::PID pitch_pid(CONTROL_DT, 1.5f, 0.05f, 0.5f, PITCH_MAX_TORQUE, 3.0f, 0.7f);
// 折叠(GM6020 电压模式)：Kp 3 V/rad
sp::PID fold_pid(CONTROL_DT, 3.0f, 0.0f, 0.05f, FOLD_MAX_TORQUE, 0.5f, 0.7f);

// ---- 运行状态（都只在本文件内读写，只有 plot_ch 对外）-------------------
float pitch_target_ = 0.0f;  // 俯仰目标角 rad
float yaw_sp_ = 0.0f;        // 偏航目标角速度 rad/s
float yaw_out_ = 0.0f;       // 偏航控制量 V
float pitch_out_ = 0.0f;     // 俯仰控制量 N·m(转矩模式)
bool fold_active_ = false;   // 折叠电机本周期是否在驱动

// 4.4.8 轨迹状态：从 disable_all 之后重新开始，相位起点记 traj_start_ms_
uint32_t traj_start_ms_ = 0;
bool traj_started_ = false;
bool traj_active_ = false;  // 本周期是否真在轨迹模式（TRAJ_ENABLE && 右拨杆上档），波形据此换参考量
float traj_ref_yaw_ = 0.0f;  // 本周期轨迹给定角 rad（只给波形看）
float traj_ref_pitch_ = 0.0f;

// 俯仰反馈（"仰为正"约定）与偏航软限位中心（上电/首次启用时锁一次）
float pitch_fb_ = 0.0f;
float yaw_center_ = 0.0f;
bool yaw_center_locked_ = false;

// 摇杆死区 + 线性重映射(过零连续，满量程仍是 ±1)
float stick(float v, float dz)
{
  if (v > dz) return (v - dz) / (1.0f - dz);
  if (v < -dz) return (v + dz) / (1.0f - dz);
  return 0.0f;
}

// 偏航软限位（机械 ±60°）：以启用时刻的 imu.yaw 为中心，到界后不许再往外走，并给一点回推
void apply_yaw_soft_limit()
{
  if (!yaw_center_locked_) return;
  const float e = sp::limit_angle(imu.yaw - yaw_center_);
  if (e > YAW_SOFT_LIMIT_RAD) {
    yaw_sp_ =
      sp::limit_min_max(yaw_sp_, -YAW_RATE_MAX, 0.0f) - YAW_LIMIT_KP * (e - YAW_SOFT_LIMIT_RAD);
  }
  else if (e < -YAW_SOFT_LIMIT_RAD) {
    yaw_sp_ =
      sp::limit_min_max(yaw_sp_, 0.0f, YAW_RATE_MAX) - YAW_LIMIT_KP * (e + YAW_SOFT_LIMIT_RAD);
  }
}

// 两条总线都发（接线在哪条线上都能动）：0x1FE = 折叠 id1 + 偏航 id2；0x200 = 俯仰 id4
// (RM_Motor::write() 按自己的 ID 往 tx_data 的槽位写，所以同帧的电机要写完再 send)
// 两条总线都发同一组指令：电机接在哪个口上都能收到（换口/飞线不用改代码）。
// 0x1FE 一帧同时带 折叠(id1 槽1) + 偏航(id2 槽2)；0x200 带 俯仰(id4 槽4)。
void send_frames()
{
  // 只在"这条总线上最近收到过电机反馈"时才发指令：
  //   - 电机接在 CAN1 或 CAN2 都行，接上电就自动开始发（不用改代码、不用换宏）；
  //   - 空着的总线一帧都不发，避免没人应答的 ACK 错误把 TEC 顶到 255 → Bus-Off。
  const uint32_t now = osKernelSysTick();
  const bool tx1 = (can1_rx_ms_ != 0u) && ((now - can1_rx_ms_) < 200u);
  const bool tx2 = (can2_rx_ms_ != 0u) && ((now - can2_rx_ms_) < 200u);

  if (tx1) {
    // 0x1FE 一帧同时带 折叠(id1 槽1) + 偏航(id2 槽2)
    for (auto & b : can1.tx_data) b = 0;
    fold_motor.write(can1.tx_data);
    yaw_motor.write(can1.tx_data);
    can1.send(fold_motor.tx_id);  // 0x1FE

    // 0x200 带 俯仰(id4 槽4)
    for (auto & b : can1.tx_data) b = 0;
    pitch_motor.write(can1.tx_data);
    can1.send(pitch_motor.tx_id);  // 0x200
  }

  if (tx2) {
    for (auto & b : can2.tx_data) b = 0;
    fold_motor.write(can2.tx_data);
    yaw_motor.write(can2.tx_data);
    can2.send(fold_motor.tx_id);  // 0x1FE

    for (auto & b : can2.tx_data) b = 0;
    pitch_motor.write(can2.tx_data);
    can2.send(pitch_motor.tx_id);  // 0x200
  }
}

// 失能：清积分 + 三个电机都发 0，并且立刻把 0 发上总线(不能只改内部指令值)
void disable_all()
{
  yaw_pid.clear();
  pitch_pid.clear();
  fold_pid.clear();
  yaw_motor.cmd(0.0f);
  pitch_motor.cmd(0.0f);
  fold_motor.cmd(0.0f);
  yaw_sp_ = 0.0f;
  yaw_out_ = 0.0f;
  pitch_out_ = 0.0f;
  fold_active_ = false;
  traj_started_ = false;  // 失能一次就从头跑轨迹，不接着上次的相位
  traj_ref_yaw_ = 0.0f;
  traj_ref_pitch_ = 0.0f;
  if (GIMBAL_ENABLE) send_frames();
}

// 失能原因码：台架上卡在哪一层，直接看 12 号通道；0 才允许出力
uint8_t gimbal_fault_code(uint32_t now_ms)
{
  if (remote.sw_r == sp::DBusSwitchMode::DOWN) return 1;  // 1) 右拨杆下档
  if (!remote.is_alive(now_ms)) return 2;                 // 2) 遥控器失联（DBus 没数据）
  if (!yaw_motor.is_alive(now_ms)) return 3;              // 3) 偏航反馈丢（0x206 / ID 2）
  if (!pitch_motor.is_alive(now_ms)) return 4;            // 4) 俯仰反馈丢（0x204 / ID 4）
  return 0;
}

void fill_plot()
{
  float fold_target = 0.0f;
  fold_active_ = fold_target_rad(&fold_target);

  plot_ch[0] = tof_distance_cm;    // 4.4.2.1：没接线时这里是 -1(无效)
  plot_ch[1] = (float)tof_status;  // 0 正常 / 1 无帧 / 2 超时 / 3 帧错 / 4 无效值或超量程
  plot_ch[2] = imu.yaw;
  plot_ch[3] = imu.pitch;
  plot_ch[4] = imu.roll;
  plot_ch[5] = traj_active_ ? traj_ref_yaw_ : yaw_sp_;  // 上档轨迹模式时是轨迹偏航角 rad
  plot_ch[6] = yaw_motor.speed;
  plot_ch[7] = yaw_out_;
  plot_ch[8] = traj_active_ ? traj_ref_pitch_ : pitch_target_;  // 上档轨迹模式时是轨迹俯仰角 rad
  plot_ch[9] = pitch_fb_;                                      // 仰为正，与 8 号目标同约定
  plot_ch[10] = pitch_out_;
  plot_ch[11] = (float)gimbal_fault_;                  // 调试：失能原因码
  plot_ch[12] = (float)gimbal_rx_seen_;                // 调试：反馈帧位图
  plot_ch[13] = (float)static_cast<int>(remote.sw_r);  // 调试：右开关档位 0 下 / 1 中 / 2 上
}
}  // namespace

// 云台电机反馈：GM6020 反馈 ID = id + 0x204，C610/RM2006 反馈 ID = id + 0x200
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  // 两条总线都要收：偏航 0x206、折叠 0x205、俯仰 0x204（send_frames 两条总线都发）
  sp::CAN * bus = (hcan == &hcan1) ? &can1 : ((hcan == &hcan2) ? &can2 : nullptr);
  if (bus == nullptr) return;

  auto now_ms = osKernelSysTick();
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    bus->recv();
    // 记下这条总线还活着：只有收到过反馈的总线才发指令（空总线不发，避免 ACK 错误累积成 Bus-Off）
    if (hcan == &hcan1) {
      can1_rx_ms_ = now_ms;
    } else {
      can2_rx_ms_ = now_ms;
    }
    // 记录收到过哪些反馈帧（0x201~0x20B 覆盖两族电机 1~8 号 ID）；波形 12 号通道显示
    //   bit0=0x201 bit1=0x202 bit2=0x203 bit3=0x204 bit4=0x205 bit5=0x206
    //   bit6=0x207 bit7=0x208 bit8=0x209 bit9=0x20A bit10=0x20B
    if (bus->rx_id >= 0x201 && bus->rx_id <= 0x20B) {
      gimbal_rx_seen_ |= static_cast<uint16_t>(1u << (bus->rx_id - 0x201));
    }
    switch (bus->rx_id) {
      case YAW_MOTOR_ID + 0x204:  // GM6020 → 0x206（CAN2）
        yaw_motor.read(bus->rx_data, now_ms);
        break;
      case PITCH_MOTOR_ID + 0x200:  // M2006/C610 → 0x204（CAN1）
        pitch_motor.read(bus->rx_data, now_ms);
        break;
      case FOLD_MOTOR_ID + 0x204:  // GM6020 → 0x205（CAN1）
        fold_motor.read(bus->rx_data, now_ms);
        break;
      default:
        break;
    }
  }
}

extern "C" void gimbal_task()
{
  if (!GIMBAL_ENABLE) {
    // 未接线 / 未使能：不碰 CAN，只把 IMU 与测距填进波形(4.4.2.1 的调试输出仍然可用)
    disable_all();
    while (true) {
      fill_plot();
      osDelay(1);
    }
  }

  can1.config();
  can1.start();
  can2.config();  // CAN2 走 bank 14 + SlaveStartFilterBank=14（sp::CAN::config 已处理）
  can2.start();
  osDelay(STARTUP_WAIT_MS);  // 等电机自检与第一帧反馈
  pitch_target_ =
    PITCH_FB_SIGN * imu.pitch;  // 上电时把俯仰目标锁在当前角("仰为正"约定)，避免一上电就窜
  yaw_center_ = imu.yaw;        // 偏航软限位中心 = 启用时刻的姿态(只锁一次)
  yaw_center_locked_ = true;

  while (true) {
    const uint32_t now_ms = osKernelSysTick();
    pitch_fb_ = PITCH_FB_SIGN * imu.pitch;  // 统一成"仰为正"约定，再进俯仰位置环

    // ---- 三层失能（与底盘工程同一套优先级：下档 > 失联 > 反馈中断）----
    gimbal_fault_ = gimbal_fault_code(now_ms);  // 1 下档 / 2 遥控失联 / 3 偏航反馈 / 4 俯仰反馈
    if (gimbal_fault_ != 0) {
      disable_all();  // 三层失能：任一原因都整机断力
    }
    else {
      // 4.4.8 触发：编译期开关打开 + 右拨杆上档（中档/上档才使能，下档在上面已失能）
      const bool traj_mode = TRAJ_ENABLE && (remote.sw_r == sp::DBusSwitchMode::UP);
      traj_active_ = traj_mode;
      if (traj_mode) {
        // ---- 4.4.8：按组委会轨迹曲线跟踪（上档进入即从轨迹起点开始）----
        if (!traj_started_) {
          traj_start_ms_ = now_ms;
          traj_started_ = true;
        }
        const TrajectoryPoint tp = trajectory_sample_at_ms(now_ms - traj_start_ms_);
        traj_ref_yaw_ = tp.yaw;
        traj_ref_pitch_ = tp.pitch;

        // 偏航：角度外环 P + 轨迹角速度前馈 → 角速度给定 → 原有速率环（复用同一套限幅）
        yaw_sp_ = sp::limit_min_max(
          TRAJ_YAW_KP * (tp.yaw - imu.yaw) + TRAJ_YAW_VEL_FF * tp.yaw_vel, -TRAJ_YAW_RATE_LIM,
          TRAJ_YAW_RATE_LIM);
        apply_yaw_soft_limit();
        yaw_pid.calc(yaw_sp_, yaw_motor.speed);
        yaw_out_ = YAW_OUT_SIGN * yaw_pid.out;
        yaw_motor.cmd(yaw_out_);

        // 俯仰：轨迹角 + 超前 lead → 限位截断 → 位置环；再叠加速度电压前馈
        pitch_target_ = sp::limit_min_max(
          tp.pitch + TRAJ_PITCH_LEAD_S * tp.pitch_vel, PITCH_MIN_RAD, PITCH_MAX_RAD);
        pitch_pid.calc(pitch_target_, pitch_fb_);
        pitch_out_ = PITCH_OUT_SIGN * sp::limit_min_max(
                                        pitch_pid.out + TRAJ_PITCH_ACC_FF * tp.pitch_acc,
                                        -PITCH_MAX_TORQUE, PITCH_MAX_TORQUE);
        pitch_motor.cmd(pitch_out_);
      }
      else {
        traj_started_ = false;  // 离开上档就复位相位：下次上档从轨迹起点重新跑
        // 偏航：右摇杆左右 → 角速度环（用电机自身编码器速度，不依赖 IMU 安装方向）
        yaw_sp_ = YAW_SIGN * stick(remote.ch_rh, STICK_DEADZONE) * YAW_RATE_MAX;
        apply_yaw_soft_limit();
        yaw_pid.calc(yaw_sp_, yaw_motor.speed);
        yaw_out_ = YAW_OUT_SIGN * yaw_pid.out;
        yaw_motor.cmd(yaw_out_);

        // 俯仰：左摇杆上下 → 目标角增量 + 位置环（IMU 角度反馈，重力负载必须闭环保持）
        float pitch_target_new = pitch_target_ + PITCH_SIGN * stick(remote.ch_lv, STICK_DEADZONE) *
                                                   PITCH_RATE_MAX * CONTROL_DT;
        pitch_target_ = sp::limit_min_max(pitch_target_new, PITCH_MIN_RAD, PITCH_MAX_RAD);
        pitch_pid.calc(pitch_target_, pitch_fb_);
        pitch_out_ = PITCH_OUT_SIGN * pitch_pid.out;
        pitch_motor.cmd(pitch_out_);
      }

      // 折叠机构(4.4.7)：本轮 FOLD_ENABLE=false，fold_target_rad() 返回 false → 一直发 0
      float fold_target = 0.0f;
      if (fold_target_rad(&fold_target)) {
        fold_pid.calc(fold_target, fold_motor.angle);
        fold_motor.cmd(fold_pid.out);
      }
      else {
        fold_pid.clear();
        fold_motor.cmd(0.0f);
      }

      send_frames();
    }

    fill_plot();
    osDelay(1);
  }
}

float gimbal_yaw_rad() { return imu.yaw; }

float gimbal_pitch_rad() { return imu.pitch; }
