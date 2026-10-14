/*
 * 第二阶段 云台 C 板：云台电机(偏航 / 俯仰 / 折叠机构)控制
 *
 * 硬件：
 *   偏航轴 GM6020(id1) / 俯仰轴 GM6020(id2) —— 电压模式(GM6020_V)，控制帧 0x1FF，反馈 0x205 / 0x206
 *   折叠机构 C610 + RM2006(id3)             —— 转矩模式，         控制帧 0x200，反馈 0x203
 *   电机挂 CAN1(PD0/PD1)；IMU 走 SPI1(bmi088 + mahony，见 imu_task.cpp)；遥控器 DT7 走 USART3(见 uart_task.cpp)
 *
 * 遥控映射(云台侧；本板自己读 DT7，与底盘 C 板互不转发)：
 *   右拨杆下档 → 云台全部电机失能(最高优先级)
 *   右摇杆左右 → 偏航角速度给定(满推 YAW_RATE_MAX rad/s)
 *   左摇杆上下 → 俯仰角速度给定(增量式位置环：松手停在当前角，因为俯仰轴有重力负载)
 *   左摇杆左右 / 拨轮 → 暂未分配，留给 4.4.8 目标轨迹跟踪
 *
 * 安全：
 *   1) 三层失能：拨杆下档 / 遥控器失联 / 电机反馈中断 —— 每周期把三个电机都发 0
 *   2) 全部输出限幅(PID 总输出 + 积分分项 + 电压 / 转矩上限)
 *   3) GIMBAL_ENABLE=false 时根本不 config/start CAN，台架上可以安全地只调测距
 */

#include <cmath>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

#include "fold_task.hpp"
#include "gimbal_task.hpp"
#include "tof_task.hpp"

// 定义在 uart_task.cpp / imu_task.cpp
extern sp::DBus remote;
extern sp::Mahony imu;

// SerialPlot 通道(定义在这里，plotter_task.cpp 里 extern)
//   0  测距(3 点中值) cm        1  测距状态码          2/3/4  IMU yaw / pitch / roll rad
//   5  偏航目标角速度          6  偏航实际角速度      7  偏航控制量 V
//   8  俯仰目标角              9  俯仰实际角(IMU)     10 俯仰控制量 V
//   11 折叠状态码              12 折叠电机目标角 rad  13 折叠电机是否被驱动
float plot_ch[14] = {0.0f};

namespace
{
constexpr float CONTROL_DT = 1e-3f;  // 控制周期 1 ms

// ---- 给定与限幅 ----------------------------------------------------------
constexpr float YAW_RATE_MAX = 6.0f;                       // rad/s：右摇杆左右满推
constexpr float PITCH_RATE_MAX = 1.5f;                     // rad/s：左摇杆上下满推
constexpr float PITCH_MIN_RAD = -30.0f * sp::SP_PI / 180.0f;   // 俯仰机械下限（先给保守值，按实际机构改）
constexpr float PITCH_MAX_RAD = 30.0f * sp::SP_PI / 180.0f;    // 俯仰机械上限
constexpr float YAW_MAX_VOLT = 24.0f;                      // GM6020 电压模式满量程 24 V
constexpr float PITCH_MAX_VOLT = 12.0f;                    // 俯仰先只给一半电压，整定时再放开
constexpr float FOLD_MAX_TORQUE = 1.0f;                    // RM2006 额定转矩约 1 N·m
constexpr float STICK_DEADZONE = 0.02f;

// 方向：现场确认后如反了就把对应 SIGN 取反（与底盘工程 FWD_DIR / SIDE_DIR / ROT_DIR 同一套路）
constexpr float YAW_SIGN = 1.0f;
constexpr float PITCH_SIGN = 1.0f;

constexpr uint32_t STARTUP_WAIT_MS = 1000;  // 等电机上电自检 + 第一帧反馈

// ---- 电机与总线 ----------------------------------------------------------
CAN_HandleTypeDef * const kMotorCan = MOTORS_ON_CAN1 ? &hcan1 : &hcan2;

sp::CAN mcan(kMotorCan);
sp::RM_Motor yaw_motor(YAW_MOTOR_ID, sp::RM_Motors::GM6020_V, 1.0f);
sp::RM_Motor pitch_motor(PITCH_MOTOR_ID, sp::RM_Motors::GM6020_V, 1.0f);
// ⚠️ 折叠机构的真实减速比取决于机构，M2006_P36 只是电机自带减速箱(36:1)；有额外减速比要改这里
sp::RM_Motor fold_motor(FOLD_MOTOR_ID, sp::RM_Motors::M2006, sp::M2006_P36);

// ---- 控制器（输出都是"电压 V" / "转矩 N·m"，由 RM_Motor::cmd() 换算成原始值）----
sp::PID yaw_pid(CONTROL_DT, 0.6f, 0.02f, 0.0f, YAW_MAX_VOLT, 6.0f, 0.7f);
sp::PID pitch_pid(CONTROL_DT, 6.0f, 0.05f, 0.2f, PITCH_MAX_VOLT, 3.0f, 0.7f);
sp::PID fold_pid(CONTROL_DT, 0.3f, 0.0f, 0.02f, FOLD_MAX_TORQUE, 0.5f, 0.7f);

// ---- 运行状态（都只在本文件内读写，只有 plot_ch 对外）-------------------
float pitch_target_ = 0.0f;   // 俯仰目标角 rad
float yaw_sp_ = 0.0f;         // 偏航目标角速度 rad/s
float yaw_volt_ = 0.0f;       // 偏航控制量 V
float pitch_volt_ = 0.0f;     // 俯仰控制量 V
bool fold_active_ = false;    // 折叠电机本周期是否在驱动

// 摇杆死区 + 线性重映射(过零连续，满量程仍是 ±1)
float stick(float v, float dz)
{
  if (v > dz) return (v - dz) / (1.0f - dz);
  if (v < -dz) return (v + dz) / (1.0f - dz);
  return 0.0f;
}

// 发两帧：0x1FF = 两个电压模式 GM6020；0x200 = 一个转矩模式 C610/RM2006
// (RM_Motor::write() 按自己的 ID 往 tx_data 的槽位写，所以同帧的电机要写完再 send)
void send_frames()
{
  for (auto & b : mcan.tx_data) b = 0;
  yaw_motor.write(mcan.tx_data);
  pitch_motor.write(mcan.tx_data);
  mcan.send(yaw_motor.tx_id);

  for (auto & b : mcan.tx_data) b = 0;
  fold_motor.write(mcan.tx_data);
  mcan.send(fold_motor.tx_id);
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
  yaw_volt_ = 0.0f;
  pitch_volt_ = 0.0f;
  fold_active_ = false;
  if (GIMBAL_ENABLE) send_frames();
}

void fill_plot()
{
  float fold_target = 0.0f;
  fold_active_ = fold_target_rad(&fold_target);

  plot_ch[0] = tof_distance_cm;      // 4.4.2.1：没接线时这里是 -1(无效)
  plot_ch[1] = (float)tof_status;    // 0 正常 / 1 无帧 / 2 超时 / 3 帧错 / 4 无效值或超量程
  plot_ch[2] = imu.yaw;
  plot_ch[3] = imu.pitch;
  plot_ch[4] = imu.roll;
  plot_ch[5] = yaw_sp_;
  plot_ch[6] = yaw_motor.speed;
  plot_ch[7] = yaw_volt_;
  plot_ch[8] = pitch_target_;
  plot_ch[9] = imu.pitch;
  plot_ch[10] = pitch_volt_;
  plot_ch[11] = (float)fold_state();
  plot_ch[12] = fold_target;
  plot_ch[13] = fold_active_ ? 1.0f : 0.0f;
}
}  // namespace

// 云台电机反馈：GM6020 反馈 ID = id + 0x204，C610/RM2006 反馈 ID = id + 0x200
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != kMotorCan) return;

  auto now_ms = osKernelSysTick();
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    mcan.recv();
    switch (mcan.rx_id) {
      case YAW_MOTOR_ID + 0x204:
        yaw_motor.read(mcan.rx_data, now_ms);
        break;
      case PITCH_MOTOR_ID + 0x204:
        pitch_motor.read(mcan.rx_data, now_ms);
        break;
      case FOLD_MOTOR_ID + 0x200:
        fold_motor.read(mcan.rx_data, now_ms);
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

  mcan.config();
  mcan.start();
  osDelay(STARTUP_WAIT_MS);          // 等电机自检与第一帧反馈
  pitch_target_ = imu.pitch;         // 上电时把俯仰目标锁在当前角，避免一上电就窜

  while (true) {
    const uint32_t now_ms = osKernelSysTick();

    // ---- 三层失能（与底盘工程同一套优先级：下档 > 失联 > 反馈中断）----
    if (remote.sw_r == sp::DBusSwitchMode::DOWN) {
      disable_all();                                    // 1) 右拨杆下档：最高优先级
    } else if (!remote.is_alive(now_ms)) {
      disable_all();                                    // 2) 遥控器失联
    } else if (!yaw_motor.is_alive(now_ms) || !pitch_motor.is_alive(now_ms)) {
      disable_all();                                    // 3) 电机反馈中断
    } else {
      // 偏航：右摇杆左右 → 角速度环（用电机自身编码器速度，不依赖 IMU 安装方向）
      yaw_sp_ = YAW_SIGN * stick(remote.ch_rh, STICK_DEADZONE) * YAW_RATE_MAX;
      yaw_pid.calc(yaw_sp_, yaw_motor.speed);
      yaw_volt_ = yaw_pid.out;
      yaw_motor.cmd(yaw_volt_);

      // 俯仰：左摇杆上下 → 目标角增量 + 位置环（IMU 角度反馈，重力负载必须闭环保持）
      float pitch_target_new =
        pitch_target_ + PITCH_SIGN * stick(remote.ch_lv, STICK_DEADZONE) * PITCH_RATE_MAX * CONTROL_DT;
      pitch_target_ = sp::limit_min_max(pitch_target_new, PITCH_MIN_RAD, PITCH_MAX_RAD);
      pitch_pid.calc(pitch_target_, imu.pitch);
      pitch_volt_ = pitch_pid.out;
      pitch_motor.cmd(pitch_volt_);

      // 折叠机构(4.4.7)：本轮 FOLD_ENABLE=false，fold_target_rad() 返回 false → 一直发 0
      float fold_target = 0.0f;
      if (fold_target_rad(&fold_target)) {
        fold_pid.calc(fold_target, fold_motor.angle);
        fold_motor.cmd(fold_pid.out);
      } else {
        fold_pid.clear();
        fold_motor.cmd(0.0f);
      }

      send_frames();
    }

    fill_plot();
    osDelay(1);
  }
}

float gimbal_yaw_rad()
{
  return imu.yaw;
}

float gimbal_pitch_rad()
{
  return imu.pitch;
}
