/*
 * 第二阶段 云台 C 板：云台电机(偏航 / 俯仰 / 折叠机构)控制
 *
 * 硬件：
 *   偏航轴 GM6020(id2，CAN2) / 折叠轴 GM6020(id1，CAN1) —— 电流模式，控制帧 0x1FE，反馈 0x206 / 0x205
 *   俯仰轴 C610 + RM2006(id4)               —— 转矩模式，         控制帧 0x200，反馈 0x204
 *   两条总线：CAN1 = 折叠 GM6020 + 俯仰 C610；CAN2 = 偏航 GM6020；IMU 走 SPI1(bmi088 + mahony，见 imu_task.cpp)；遥控器 DT7 走 USART3(见 uart_task.cpp)
 *
 * 遥控映射(云台侧；本板自己读 DT7，与底盘 C 板互不转发)：
 *   右拨杆下档 → 云台全部电机失能(最高优先级)
 *   右摇杆左右 → 偏航角速度给定(满推 YAW_RATE_MAX rad/s)
 *   左摇杆上下 → 俯仰角速度给定(增量式位置环：松手停在当前角，因为俯仰轴有重力负载)
 *   左摇杆左右 / 拨轮 → 暂未分配
 *   4.4.8 目标轨迹跟踪 → TRAJ_ENABLE=true 且右拨杆打到 TRAJ_GEAR（默认上档）时接管偏航/俯仰给定
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
// 收到过的反馈帧位图（调试）：bit(n) = 收到过 0x201+n
//   bit0 0x201            bit1 0x202            bit2 0x203 = C610 id3
//   bit3 0x204 = C610 id4(俯仰)   bit4 0x205 = GM6020 id1(折叠)   bit5 0x206 = GM6020 id2(偏航)
//   例：只有折叠+俯仰 = 8+16 = 24；再接上底部偏航 = 8+16+32 = 56
uint16_t gimbal_rx_seen_ = 0;

// 三路反馈帧的到达时刻（ISR 里记），用来自己判数据新不新鲜。
// 为什么不用 RM_Motor::is_alive()：它算的是无符号差值 now - last_read_ms_ < 100，
// 而 CAN 中断打的时间戳可能比主循环这一拍采到的 now_ms 新 1 ms，相减会回绕成 0xFFFFFFFF，
// 于是明明在收帧却被判成失联（这个坑以前在底盘上踩过，中间件不许改，就在应用层自己判）。
uint32_t yaw_rx_ms_ = 0;
uint32_t pitch_rx_ms_ = 0;
uint32_t fold_rx_ms_ = 0;
uint32_t can1_rx_ms_ = 0;  // CAN1 最近一次收到电机反馈（0 = 从未收到）
uint32_t can2_rx_ms_ = 0;  // CAN2 同上

namespace
{
constexpr float CONTROL_DT = 1e-3f;  // 控制周期 1 ms

// ---- 给定与限幅 ----------------------------------------------------------
constexpr float YAW_RATE_MAX = 1.5f;    // rad/s：右摇杆左右满推（事故后 6.0→1.5，约 86 °/s）
constexpr float PITCH_RATE_MAX = 0.8f;  // rad/s：左摇杆上下满推（事故后 1.5→0.8，约 46 °/s）
// 俯仰机械限位的唯一出处是 gimbal_task.hpp 里的 PITCH_LIMIT_MIN/MAX_RAD（值待建模实测）
constexpr float PITCH_MIN_RAD = PITCH_LIMIT_MIN_RAD;  // 俯仰机械下限
constexpr float PITCH_MAX_RAD = PITCH_LIMIT_MAX_RAD;  // 俯仰机械上限
constexpr float YAW_MAX_TORQUE = 0.3f;  // GM6020 满量程 2.223 N·m；事故后收到 0.3，机构验证过再放开
constexpr float PITCH_MAX_TORQUE =
  0.3f;                                  // 事故后 1.8 → 0.5 → 0.3：PLA 件扛不住，换件验证后再逐步加
constexpr float FOLD_MAX_TORQUE = 1.2f;  // 折叠 GM6020 上限（驱动层封顶 2.223 N·m）；事故后 2.2→1.2
constexpr bool FOLD_TORQUE_PLOT = false;  // 5 号通道显折叠输出转矩 N·m（偏航未启用时调试用）
constexpr bool YAW_SPEED_PLOT =
  true;  // 4 号通道改显偏航实际角速度 rad/s（调偏航环时用；不调了改回 false 看 roll）

// ---- 安全联锁（2026/10/21 事故后加）----------------------------------------
// 事故：折叠被重力压下来 → 上段倒下 → IMU 姿态剧变 → 俯仰环还在按轨迹纠偏 →
//       3D 打印件承受异常扭矩被扯断。原则：机构姿态一旦异常就"撒手"(0 转矩)，
//       绝不用大转矩去救；并且折叠不在展开位时根本不许进轨迹跟踪。
constexpr bool ATTITUDE_GUARD_ENABLE = true;          // 姿态异常 → 立即失能（原因码 5）
constexpr float GUARD_ROLL_MAX_RAD = 0.261799f;       // 15°：横滚超限说明上段倒了/机构翻了
constexpr float GUARD_PITCH_ERR_MAX_RAD = 0.349066f;  // 20°：跟随差这么多且已在出力 → 被判卡住
constexpr uint32_t GUARD_TRIP_MS = 50u;               // 连续超限 50 ms 才触发（滤掉瞬时冲击）
constexpr bool TRAJ_REQUIRE_FOLD_DEPLOY = true;       // 折叠不在展开位(±5°)就不许进轨迹
constexpr float TRAJ_FOLD_TOL_RAD = 0.0872665f;
// 抗重力前馈：俯仰臂+头绕折叠轴的重力矩（按建模估算：展开姿态约 0.2 N·m，
// 转到"臂水平"时最大约 1.0 N·m）。**默认关（0）**：常值前馈的方向必须先验证，
// 符号反了会变成"压着往下"，比不开更糟（台上 FF=1.5 仍下沉就是符号问题）。
// 要用就每次只改 FOLD_GRAVITY_FF_NM，同时看 5 号通道(CH5) 的转矩和下沉方向：
//   FF 加大后下沉更快 → FOLD_FF_SIGN 改 -1；
//   FF 加大后能顶住或往上顶 → 符号对了，把 FF 填成 CH5 稳定下来的那个数值。
constexpr float FOLD_GRAVITY_FF_NM = 0.0f;
constexpr float FOLD_FF_SIGN = 1.0f;
// 折叠速度规划：不给目标角阶跃，而是让目标角按限速/限加速度爬过去
// 0.8 rad/s = 46 deg/s；80 deg（1.396 rad）的行程约 1.8 s 走完。还是太快就把 FOLD_SPEED_MAX 调小。
// 右拨杆中档 = 折叠展开（现场要求）：中档时忽略测距，直接把折叠臂摆到展开位；
// 其它档位仍按测距自动折叠（AUTO），台架可用 FOLD_CMD 强制收/展。
constexpr bool FOLD_MID_DEPLOY = false;
constexpr float FOLD_SPEED_MAX = 0.8f;   // rad/s：折叠目标角变化率上限（事故后 1.2→0.8，约 46 °/s）
constexpr float FOLD_ACC_MAX = 1.5f;     // rad/s²：折叠目标角加速度上限（事故后 3.0→1.5）
constexpr float FOLD_SPEED_GAIN = 4.0f;  // 1/s：接近目标时按这个增益减速
// 折叠轴软限位：以上电锁存角为中心，顺/逆时针各 60°（机械限位由这一层兜住）
// 折叠软限位窗口由 fold_task.hpp 的 FOLD_LIMIT_LO_RAD / FOLD_LIMIT_HI_RAD 给出（相对上电零位）
// ---- 使能瞬间的柔和处理（避免切档时猛拉/猛冲）----
// 失能期间机构会被重力放下来；重新使能时若还按“上电时的设定角”闭环，误差很大 → 第一拍就满转矩。
// 做法：使能第一拍把设定角/规划点“就地接管”成当前实际角，再加上出力变化率限制。
constexpr float PITCH_OUT_SLEW = 3.0f;    // N·m/s：俯仰出力变化率上限（3 → 0.3 N·m 约 100 ms）
constexpr float FOLD_TORQUE_SLEW = 8.0f;  // N·m/s：折叠出力变化率上限（8 → 1.2 N·m 约 150 ms）
constexpr float YAW_OUT_SLEW = 8.0f;      // N·m/s：偏航出力变化率上限（事故后新加，防阶跃）
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
// ⚠ 2026/10/21 事故后默认关掉：等折叠保持/结构修好、手托验证过再改 true
// 4.4.8 编译期总开关。⚠ 2026/10/21 事故后默认关掉：
// 等折叠保持与结构修好、并手托验证过再改 true。
constexpr bool TRAJ_ENABLE = true;  // 编译期总开关；运行时还要把右拨杆打到 TRAJ_GEAR 才接管
// 只看不驱动：先上档看波形上的目标轨迹，确认无误再改成 false 让电机真跟
constexpr bool TRAJ_DRYRUN = false;
// 预演：true = 不看遥控器也不看拨杆，上电就循环播放参考曲线（只给波形看；配合 TRAJ_DRYRUN 电机不出力）
constexpr bool TRAJ_PREVIEW = false;
constexpr float TRAJ_YAW_KP = 3.0f;      // 1/s        角度误差 → 角速度给定
constexpr float TRAJ_YAW_VEL_FF = 1.0f;  // 轨迹角速度前馈系数（1.0 = 直接叠加）
constexpr float TRAJ_YAW_RATE_LIM =
  1.5f;  // rad/s      跟踪时角速度上限（要 >= AMP_SCALE x 轨迹峰值 7.15 = 1.07）
constexpr float TRAJ_PITCH_LEAD_S = 0.03f;   // s          参考角超前量（事故后 0.06→0.03，别抢跑）
constexpr float TRAJ_PITCH_ACC_FF = 0.004f;  // N·m/(rad/s²) 加速度转矩前馈（标定项，默认 0）
constexpr bool TRAJ_ANCHOR_ON_ENTER = true;  // 上档瞬间把轨迹原点对齐到当前姿态（避免参考阶跃）
constexpr float TRAJ_AMP_SCALE =
  0.15f;  // 轨迹幅度缩放：事故后 0.3→0.15（约 ±4.5° 俯仰 / ±3.4° 偏航）
constexpr float TRAJ_REF_SLEW =
  1.5f;  // rad/s 俯仰参考角变化率上限（要 >= AMP_SCALE x 轨迹峰值 8.31 = 1.25）
constexpr sp::DBusSwitchMode TRAJ_GEAR =
  sp::DBusSwitchMode::UP;  // 轨迹挂在哪个档（UP=上档，MID=中档）

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
sp::PID yaw_pid(CONTROL_DT, 0.6f, 0.2f, 0.0f, YAW_MAX_TORQUE, YAW_MAX_TORQUE, 0.7f);
// 速度环：Ki 0.02→0.2（1 ms 周期下 0.02 等于没有积分）；max_iout 收到 = max_out，否则积分会 windup
// 俯仰(M2006 转矩模式)：Kp 1.5 N·m/rad，Ki 用来扛重力
sp::PID pitch_pid(CONTROL_DT, 3.0f, 5.0f, 0.15f, PITCH_MAX_TORQUE, PITCH_MAX_TORQUE, 0.7f);
// 位置环：Ki 0.05→5（1 ms 周期下 0.05 要几十秒才攒出力矩）；到位后缓慢爬行/来回找就把 Ki 降回 2
// 折叠(GM6020 电压模式)：Kp 3 V/rad
sp::PID fold_pid(
  CONTROL_DT, 8.0f, 8.0f, 0.20f, FOLD_MAX_TORQUE, FOLD_MAX_TORQUE,
  0.7f);  // Kp 硬顶 + Ki 扛重力；max_iout=max_out 防 windup

// ---- 运行状态（都只在本文件内读写，只有 plot_ch 对外）-------------------
float pitch_target_ = 0.0f;  // 俯仰目标角 rad
float yaw_sp_ = 0.0f;        // 偏航目标角速度 rad/s
float yaw_out_ = 0.0f;       // 偏航控制量 V
float pitch_out_ = 0.0f;     // 俯仰控制量 N·m(转矩模式)
uint32_t guard_ms_ = 0;      // 姿态联锁：超限持续了多少个控制周期（1 ms/拍）
bool fold_active_ = false;
// 无折叠指令时的"保持角"：上电拿到反馈后锁存一次，之后一直闭环顶住（不发 0 转矩，避免重力下垂）
float fold_hold_rad_ = 0.0f;
bool fold_hold_valid_ = false;
float fold_center_rad_ = 0.0f;  // 折叠轴软限位中心（上电锁存角）
bool fold_center_valid_ = false;
float fold_sp_rad_ = 0.0f;  // 速度规划后的折叠目标角（rad）
float fold_sp_vel_ = 0.0f;  // 速度规划中的目标角速度（rad/s）

// 使能瞬间的对齐标志：失能时置位，重新使能的第一拍把设定/规划点对到当前实际角
bool pitch_takeover_ = true;
bool fold_resync_ = true;

// 4.4.8 轨迹状态：从 disable_all 之后重新开始，相位起点记 traj_start_ms_
uint32_t traj_start_ms_ = 0;
bool traj_started_ = false;
bool traj_active_ = false;  // 本周期是否真在轨迹模式（TRAJ_ENABLE && 右拨杆上档），波形据此换参考量
float traj_ref_yaw_ = 0.0f;  // 本周期轨迹给定角 rad（只给波形看）
float traj_ref_pitch_ = 0.0f;
// 上档瞬间的基准：把轨迹映射成"从当前姿态出发、幅度缩放后的相对运动"
bool traj_anchor_valid_ = false;
float traj_anchor_yaw_ = 0.0f;    // 上档时的 imu.yaw
float traj_anchor_pitch_ = 0.0f;  // 上档时的 pitch_fb_
float traj_tp0_yaw_ = 0.0f;       // 轨迹起点角
float traj_tp0_pitch_ = 0.0f;

// 俯仰反馈（"仰为正"约定）
float pitch_fb_ = 0.0f;
float fold_out_ = 0.0f;  // 折叠输出转矩 N·m（调试通道用）

// ---- 上电零位：第一次收到反馈时的角度记为 0 ----
// 折叠/偏航的目标角、软限位、波形都相对它：上电姿态就是零位，不用管编码器出厂零位在哪。
// 只在每次上电锁一次；失能、重新使能都不会重锁（软限位窗口因此不会漂）。
float fold_zero_rad_ = 0.0f;
bool fold_zero_valid_ = false;
float yaw_zero_rad_ = 0.0f;
bool yaw_zero_valid_ = false;

// 相对上电零位的电机角（rad）
float fold_angle()
{
  return fold_zero_valid_ ? (fold_motor.angle - fold_zero_rad_) : fold_motor.angle;
}

float yaw_angle() { return yaw_zero_valid_ ? (yaw_motor.angle - yaw_zero_rad_) : yaw_motor.angle; }

// 摇杆死区 + 线性重映射(过零连续，满量程仍是 ±1)
float stick(float v, float dz)
{
  if (v > dz) return (v - dz) / (1.0f - dz);
  if (v < -dz) return (v + dz) / (1.0f - dz);
  return 0.0f;
}

// 偏航软限位（机械 ±60°）：以"上电零位"为中心（GM6020 直驱绝对编码器，不漂），
// 到界后不许再往外走，并给一点回推
void apply_yaw_soft_limit()
{
  if (!yaw_zero_valid_) return;
  const float e = yaw_angle();
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
    if (YAW_ENABLE) yaw_motor.write(can1.tx_data);  // YAW_ENABLE=false 时该槽保持 0
    can1.send(fold_motor.tx_id);                    // 0x1FE

    // 0x200 带 俯仰(id4 槽4)
    for (auto & b : can1.tx_data) b = 0;
    pitch_motor.write(can1.tx_data);
    can1.send(pitch_motor.tx_id);  // 0x200
  }

  if (tx2) {
    for (auto & b : can2.tx_data) b = 0;
    fold_motor.write(can2.tx_data);
    if (YAW_ENABLE) yaw_motor.write(can2.tx_data);  // YAW_ENABLE=false 时该槽保持 0
    can2.send(fold_motor.tx_id);                    // 0x1FE

    for (auto & b : can2.tx_data) b = 0;
    pitch_motor.write(can2.tx_data);
    can2.send(pitch_motor.tx_id);  // 0x200
  }
}

// 输出限速：每个周期最多变化 step（用来消掉使能/切档瞬间的转矩阶跃）
float slew_toward(float cur, float goal, float step)
{
  if (goal > cur + step) return cur + step;
  if (goal < cur - step) return cur - step;
  return goal;
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
  pitch_takeover_ = true;  // 下次使能就地接管，不猛拉回上电时的设定角
  fold_resync_ = true;     // 下次使能把折叠规划点对到实际角
  fold_active_ = false;
  fold_hold_valid_ = false;    // 下档失能时不允许顶住（安全要求），下次使能重新锁存当前位置
  fold_center_valid_ = false;  // 软限位中心也重新锁存
  traj_started_ = false;       // 失能一次就从头跑轨迹，不接着上次的相位
  traj_ref_yaw_ = 0.0f;
  traj_ref_pitch_ = 0.0f;
  if (GIMBAL_ENABLE) send_frames();
}

// 失能原因码：台架上卡在哪一层，直接看 12 号通道；0 才允许出力
// 这一路反馈最近 100 ms 内有没有更新（带符号差值，容忍时间戳比 now_ms 略新）
bool rx_fresh(uint32_t stamp_ms, uint32_t now_ms)
{
  if (stamp_ms == 0u) return false;  // 一帧都还没收到
  const int32_t dt = static_cast<int32_t>(now_ms - stamp_ms);
  return dt > -50 && dt < 100;
}

uint8_t gimbal_fault_code(uint32_t now_ms)
{
  // 先判“有没有数据”：sw_r 的零值等于 DOWN，没数据时会被误判成“拨杆下档”
  if (!remote.is_alive(now_ms)) return 2;                     // 2) 遥控器失联（DBus 没数据）
  if (remote.sw_r == sp::DBusSwitchMode::DOWN) return 1;      // 1) 右拨杆下档
  if (YAW_ENABLE && !rx_fresh(yaw_rx_ms_, now_ms)) return 3;  // 3) 偏航反馈丢（0x206 / ID 2）
  if (!rx_fresh(pitch_rx_ms_, now_ms)) return 4;              // 4) 俯仰反馈丢（0x204 / ID 4）
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
  plot_ch[4] = YAW_SPEED_PLOT ? yaw_motor.speed : imu.roll;  // 调偏航环时看实际角速度
  // 5 号通道：默认偏航角速度指令；FOLD_TORQUE_PLOT=true 时改显折叠输出转矩 N·m
  // （调试折叠下沉用：稳定之后这个数就是该姿态需要的保持力矩）
  plot_ch[5] = FOLD_TORQUE_PLOT ? fold_out_ : (traj_active_ ? traj_ref_yaw_ : yaw_sp_);
  plot_ch[6] = fold_sp_rad_;                                    // 折叠目标角（已限速规划）rad
  plot_ch[7] = fold_angle();                                    // 折叠实际角（相对上电零位）rad
  plot_ch[8] = traj_active_ ? traj_ref_pitch_ : pitch_target_;  // 上档轨迹模式时是轨迹俯仰角 rad
  plot_ch[9] = pitch_fb_;                                       // 仰为正，与 8 号目标同约定
  plot_ch[10] = pitch_out_;
  plot_ch[11] = (float)gimbal_fault_;    // 调试：失能原因码
  plot_ch[12] = (float)gimbal_rx_seen_;  // 调试：反馈帧位图
  // 调试：0 没数据 / 1 下 / 2 中 / 3 上（+1 是为了把“没数据”和“下档”分开）
  plot_ch[13] =
    remote.is_alive(osKernelSysTick()) ? (float)(1 + static_cast<int>(remote.sw_r)) : 0.0f;
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
    }
    else {
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
        yaw_rx_ms_ = now_ms;
        yaw_motor.read(bus->rx_data, now_ms);
        if (!yaw_zero_valid_) {  // 上电零位：第一次拿到反馈就锁存
          yaw_zero_rad_ = yaw_motor.angle;
          yaw_zero_valid_ = true;
        }
        break;
      case PITCH_MOTOR_ID + 0x200:  // M2006/C610 → 0x204（CAN1）
        pitch_rx_ms_ = now_ms;
        pitch_motor.read(bus->rx_data, now_ms);
        break;
      case FOLD_MOTOR_ID + 0x204:  // GM6020 → 0x205（CAN1）
        fold_rx_ms_ = now_ms;
        fold_motor.read(bus->rx_data, now_ms);
        if (!fold_zero_valid_) {  // 上电零位：第一次拿到反馈就锁存
          fold_zero_rad_ = fold_motor.angle;
          fold_zero_valid_ = true;
        }
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
  // 偏航软限位中心 = 上电零位（编码器零位由第一帧反馈锁存，不依赖 IMU 漂移）

  while (true) {
    const uint32_t now_ms = osKernelSysTick();
    pitch_fb_ = PITCH_FB_SIGN * imu.pitch;  // 统一成"仰为正"约定，再进俯仰位置环

    // ---- 三层失能（与底盘工程同一套优先级：下档 > 失联 > 反馈中断）----
    gimbal_fault_ = gimbal_fault_code(now_ms);  // 1 下档 / 2 遥控失联 / 3 偏航反馈 / 4 俯仰反馈

    // ---- 姿态联锁（事故后加）：横滚超限 / 跟随精度崩了且已在出力 → 判机构异常，撒手 ----
    if (
      ATTITUDE_GUARD_ENABLE &&
      (fabsf(imu.roll) > GUARD_ROLL_MAX_RAD ||
       (fabsf(pitch_target_ - pitch_fb_) > GUARD_PITCH_ERR_MAX_RAD && fabsf(pitch_out_) > 0.2f))) {
      if (guard_ms_ < GUARD_TRIP_MS) ++guard_ms_;
    }
    else {
      guard_ms_ = 0;
    }
    if (gimbal_fault_ == 0 && guard_ms_ >= GUARD_TRIP_MS) gimbal_fault_ = 5;  // 5 = 姿态/跟随异常
    if (gimbal_fault_ != 0) {
      disable_all();  // 三层失能：任一原因都整机断力
    }
    else {
      // 4.4.8 触发：编译期开关打开 + 右拨杆上档（中档/上档才使能，下档在上面已失能）
      // 折叠不在展开位（掉了 / 没折回来）时不许进轨迹：否则上段倒下 + 轨迹纠偏会把结构扯坏
      const bool fold_deployed_ok =
        (!TRAJ_REQUIRE_FOLD_DEPLOY) || (fabsf(fold_angle() - FOLD_DEPLOY_RAD) < TRAJ_FOLD_TOL_RAD);
      const bool traj_mode = TRAJ_ENABLE && (remote.sw_r == TRAJ_GEAR) && fold_deployed_ok;
      traj_active_ = traj_mode;
      if (traj_mode) {
        // ---- 4.4.8：按组委会轨迹曲线跟踪 ----
        // 上档瞬间不直接跳到轨迹起点（起点是 yaw +18° / pitch -30°，一进去就是 0.5 rad 阶跃，
        // 位置环会立刻满转矩砸过去）。做法：把轨迹当作"相对当前姿态的偏移"，
        // 参考角 = 上档时的姿态 + TRAJ_AMP_SCALE · (轨迹角 - 轨迹起点角)，天然连续。
        if (!traj_started_) {
          traj_start_ms_ = now_ms;
          traj_started_ = true;
          const TrajectoryPoint tp0 = trajectory_sample_at_ms(0);
          traj_anchor_yaw_ = imu.yaw;
          traj_anchor_pitch_ = sp::limit_min_max(pitch_fb_, PITCH_MIN_RAD, PITCH_MAX_RAD);
          traj_tp0_yaw_ = tp0.yaw;
          traj_tp0_pitch_ = tp0.pitch;
          traj_anchor_valid_ = true;
        }
        const TrajectoryPoint tp = trajectory_sample_at_ms(now_ms - traj_start_ms_);
        float yaw_ref = tp.yaw;
        float pitch_ref = tp.pitch;
        if (TRAJ_ANCHOR_ON_ENTER && traj_anchor_valid_) {
          yaw_ref = traj_anchor_yaw_ + TRAJ_AMP_SCALE * (tp.yaw - traj_tp0_yaw_);
          pitch_ref = traj_anchor_pitch_ + TRAJ_AMP_SCALE * (tp.pitch - traj_tp0_pitch_);
        }
        traj_ref_yaw_ = yaw_ref;
        traj_ref_pitch_ = pitch_ref;

        if (TRAJ_DRYRUN) {
          // 只看参考曲线：两个轴都不出力，波形 5/8 号仍然是轨迹角，9 号是实际角
          yaw_sp_ = 0.0f;
          yaw_out_ = 0.0f;
          yaw_motor.cmd(0.0f);
          pitch_target_ = pitch_fb_;
          pitch_out_ = 0.0f;
          pitch_motor.cmd(0.0f);
        }
        else {
          // 偏航：角度外环 P + 轨迹角速度前馈 → 角速度给定 → 原有速率环（复用同一套限幅）
          yaw_sp_ = sp::limit_min_max(
            TRAJ_YAW_KP * (yaw_ref - imu.yaw) + TRAJ_YAW_VEL_FF * TRAJ_AMP_SCALE * tp.yaw_vel,
            -TRAJ_YAW_RATE_LIM, TRAJ_YAW_RATE_LIM);
          apply_yaw_soft_limit();
          yaw_pid.calc(yaw_sp_, yaw_motor.speed);
          yaw_out_ = slew_toward(yaw_out_, YAW_OUT_SIGN * yaw_pid.out, YAW_OUT_SLEW * CONTROL_DT);
          yaw_motor.cmd(yaw_out_);

          // 俯仰：参考角 + 超前 lead → 限位截断 → 变化率限制 → 位置环；再叠加速度转矩前馈
          const float pitch_target_goal = sp::limit_min_max(
            pitch_ref + TRAJ_PITCH_LEAD_S * TRAJ_AMP_SCALE * tp.pitch_vel, PITCH_MIN_RAD,
            PITCH_MAX_RAD);
          pitch_target_ = slew_toward(pitch_target_, pitch_target_goal, TRAJ_REF_SLEW * CONTROL_DT);
          pitch_pid.calc(pitch_target_, pitch_fb_);
          const float pitch_out_goal =
            PITCH_OUT_SIGN * sp::limit_min_max(
                               pitch_pid.out + TRAJ_PITCH_ACC_FF * TRAJ_AMP_SCALE * tp.pitch_acc,
                               -PITCH_MAX_TORQUE, PITCH_MAX_TORQUE);
          pitch_out_ = slew_toward(pitch_out_, pitch_out_goal, PITCH_OUT_SLEW * CONTROL_DT);
          pitch_motor.cmd(pitch_out_);
        }
      }
      else {
        traj_started_ = false;  // 离开上档就复位相位：下次上档从轨迹起点重新跑
        // 偏航：右摇杆左右 → 角速度环（用电机自身编码器速度，不依赖 IMU 安装方向）
        yaw_sp_ = YAW_SIGN * stick(remote.ch_rh, STICK_DEADZONE) * YAW_RATE_MAX;
        apply_yaw_soft_limit();
        yaw_pid.calc(yaw_sp_, yaw_motor.speed);
        yaw_out_ = slew_toward(yaw_out_, YAW_OUT_SIGN * yaw_pid.out, YAW_OUT_SLEW * CONTROL_DT);
        yaw_motor.cmd(yaw_out_);

        // 俯仰：左摇杆上下 → 目标角增量 + 位置环（IMU 角度反馈，重力负载必须闭环保持）
        // 使能第一拍：设定角就地接管成当前角（失能期间被重力放下了也不会一使能就满转矩猛拉）
        if (pitch_takeover_) {
          pitch_target_ = sp::limit_min_max(pitch_fb_, PITCH_MIN_RAD, PITCH_MAX_RAD);
          pitch_pid.clear();
          pitch_takeover_ = false;
        }
        float pitch_target_new = pitch_target_ + PITCH_SIGN * stick(remote.ch_lv, STICK_DEADZONE) *
                                                   PITCH_RATE_MAX * CONTROL_DT;
        pitch_target_ = sp::limit_min_max(pitch_target_new, PITCH_MIN_RAD, PITCH_MAX_RAD);
        pitch_pid.calc(pitch_target_, pitch_fb_);
        const float pitch_out_goal = PITCH_OUT_SIGN * pitch_pid.out;
        pitch_out_ = slew_toward(pitch_out_, pitch_out_goal, PITCH_OUT_SLEW * CONTROL_DT);
        pitch_motor.cmd(pitch_out_);
      }

      // 折叠机构(4.4.7)：目标角先过速度规划，再进位置环（避免机构被瞬间拉到位）
      // 没有指令时 **不能发 0 转矩**（机构会被重力拉下来），而是保持在上电锁存的角度上：
      //   有指令 → 保持点跟着指令的终值走（收/展完成后就停在那）；
      //   无指令 → 闭环顶住 fold_hold_rad_；上电后第一次收到折叠反馈时才锁存，
      //            还没收到反馈就先不出力（此时电机本来也没上电）。
      float fold_target = 0.0f;
      float fold_goal;
      // 中档 = 展开：优先于测距/台架指令
      const bool mid_deploy = FOLD_MID_DEPLOY && (remote.sw_r == sp::DBusSwitchMode::MID);
      if (mid_deploy) {
        fold_target = FOLD_DEPLOY_RAD;
      }
      // 轨迹跟踪期间把折叠锁在展开位：跑轨迹时不许被测距触发收折（两者会打架）
      if (traj_active_) {
        fold_target = FOLD_DEPLOY_RAD;
      }
      if (mid_deploy || traj_active_ || fold_target_rad(&fold_target)) {
        fold_hold_rad_ = fold_target;
        fold_hold_valid_ = true;
        fold_goal = fold_target;
      }
      else {
        if (!fold_hold_valid_ && rx_fresh(fold_rx_ms_, now_ms)) {
          fold_hold_rad_ = fold_angle();    // 上电后第一次拿到反馈：把当前角度锁成保持点
          fold_center_rad_ = fold_angle();  // 软限位中心 = 上电姿态
          fold_center_valid_ = true;
          fold_hold_valid_ = true;
        }
        fold_goal = fold_hold_rad_;
      }
      // 软限位：目标和规划点都夹在 [FOLD_LIMIT_LO_RAD, FOLD_LIMIT_HI_RAD]（相对上电零位）
      fold_goal = sp::limit_min_max(fold_goal, FOLD_LIMIT_LO_RAD, FOLD_LIMIT_HI_RAD);
      if (fold_hold_valid_) {
        // 重新使能第一拍：规划点就地接管成实际角，避免从旧目标一路冲过来
        if (fold_resync_ && rx_fresh(fold_rx_ms_, now_ms)) {
          fold_sp_rad_ = fold_angle();
          fold_sp_vel_ = 0.0f;
          fold_resync_ = false;
        }
        const float fold_err = fold_goal - fold_sp_rad_;
        const float fold_v_des =
          sp::limit_min_max(fold_err * FOLD_SPEED_GAIN, -FOLD_SPEED_MAX, FOLD_SPEED_MAX);
        fold_sp_vel_ = sp::limit_min_max(
          fold_v_des, fold_sp_vel_ - FOLD_ACC_MAX * CONTROL_DT,
          fold_sp_vel_ + FOLD_ACC_MAX * CONTROL_DT);
        fold_sp_rad_ += fold_sp_vel_ * CONTROL_DT;
        // 规划点也夹在软限位里，别让积分绕过
        fold_sp_rad_ = sp::limit_min_max(fold_sp_rad_, FOLD_LIMIT_LO_RAD, FOLD_LIMIT_HI_RAD);
        fold_pid.calc(fold_sp_rad_, fold_angle());
        // 前馈扛重力 + PID 修偏差，一起限幅到 FOLD_MAX_TORQUE
        // 抗重力前馈：折叠角越大（越接近水平）重力矩越大，用 sin 形状补偿；静差再由积分修
        const float fold_ff = FOLD_FF_SIGN * FOLD_GRAVITY_FF_NM * sinf(fold_angle());
        const float fold_out_goal =
          sp::limit_min_max(fold_pid.out + fold_ff, -FOLD_MAX_TORQUE, FOLD_MAX_TORQUE);
        fold_out_ = slew_toward(fold_out_, fold_out_goal, FOLD_TORQUE_SLEW * CONTROL_DT);
        fold_motor.cmd(fold_out_);
      }
      else {
        fold_pid.clear();
        fold_sp_rad_ = fold_angle();  // 还没有反馈：规划点贴住实际角，先不出力
        fold_sp_vel_ = 0.0f;
        fold_out_ = 0.0f;
        fold_motor.cmd(0.0f);
      }

      send_frames();
    }

    // 轨迹预演：不管遥控器是否在线、拨杆在哪一档，都把参考曲线算出来喂给波形
    if (TRAJ_PREVIEW && TRAJ_ENABLE) {
      if (!traj_started_) {
        traj_start_ms_ = now_ms;
        traj_started_ = true;
      }
      const TrajectoryPoint tp_prev = trajectory_sample_at_ms(now_ms - traj_start_ms_);
      traj_ref_yaw_ = tp_prev.yaw;
      traj_ref_pitch_ = tp_prev.pitch;
      traj_active_ = true;
    }

    fill_plot();
    osDelay(1);
  }
}
