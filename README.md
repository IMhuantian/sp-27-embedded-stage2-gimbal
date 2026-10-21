# cboard_gimbal（云台 C 板）

RoboMaster 开发板 C 型（STM32F407IGHx）上的**云台**嵌入式工程。
当前进度：**第二阶段 · 云台**——三台电机通信已通（偏航 GM6020 / 折叠 GM6020 / 俯仰 C610+RM2006）、红外测距（4.4.2.1）已完成；4.4.7 自动折叠状态机已启用并实机动作过（上电即保持位置）；4.4.8 轨迹骨架已接入（`TRAJ_ENABLE`，上档触发，默认只预演不出力）。

## 两块 C 板的分工

| 工程 | 目录 | 职责（规则书） | 遥控器 | 管的电机 |
|---|---|---|---|---|
| 底盘 C 板 | `..\files\stage_2` | 4.4.2.2 遥控器与底盘控制、4.4.2.3 功率控制、功率计 | 自己读 DT7 | 4×RM3508 + C620（CAN1） |
| **云台 C 板（本工程）** | `..\files\stage_2_gimbal` | 4.4.2.1 红外测距 + C 板基本功能（LED / 蜂鸣器）、4.4.7 云台自动折叠、4.4.8 目标轨迹跟踪 | 自己读 DT7 | 偏航 GM6020（CAN2）、折叠 GM6020（CAN1）、俯仰 C610 + RM2006（CAN1） |

两块板**各自独立**：各自读自己的 DT7 接收机、各自驱动自己的电机，不互相转发遥控指令，
也不共享串口数据。本工程**不含**底盘运动解算、功率控制与功率计解析（那些在底盘工程里）。

## 功能

- 蜂鸣器提示音、RGB 流水灯（C 板基本功能）
- IMU(BMI088) 采集 + Mahony 姿态解算，三轴姿态角经 SerialPlot 输出（USART1 / 921600 / 帧头 `AA BB`）
- DT7 遥控器通讯
- **GY-56 红外测距读取（4.4.2.1）**：动态帧长（8~13 字节）、3 点中值滤波、帧头/校验/无效值/超量程/超时分别计数并给状态码，对外提供 `tof_get_distance()` 供 4.4.7 折叠逻辑调用
- **云台电机控制框架**：偏航（GM6020 **电流模式**，角速度环，CAN2）+ 折叠（GM6020 **电流模式**，位置环，CAN1）+ 俯仰（C610 + RM2006 转矩模式，增量式位置环，CAN1）接口
- **4.4.7 折叠决策**：独立任务、阈值 + 回差 + 去抖 + 动作超时保护；折叠轴目标角带速度规划，**无指令时闭环保持当前位置**（不会因重力下垂）；折叠与偏航的**上电姿态即零位**（`fold_angle()` / `yaw_angle()`）

## 任务

| 任务 | 优先级 | 栈(字) | 入口 | 干什么 |
|---|---|---|---|---|
| defaultTask | Normal | 128 | StartDefaultTask | USB 初始化 |
| ImuTask | AboveNormal | 512 | imu_task | 1kHz 采 IMU 并解算 |
| UartTask | Normal | 256 | uart_task | DBus 接收 + 串口回调转发（USART3 / USART6） |
| TofTask | Normal | 256 | tof_task | 上电等 1 s 自检 → 发配置命令 → 10 ms 巡检 GY-56（动态帧长 + 中值 + 超时判无效） |
| **GimbalTask** | AboveNormal | 256 | gimbal_task | 1kHz 云台控制：三层失能 + 偏航角速度环 + 俯仰位置环 + 折叠电机位置环 |
| **FoldTask** | Normal | 256 | fold_task | 10 ms 折叠决策（4.4.7 要求独立任务；本轮只跑骨架） |
| PlotterTask | Low | 128 | plotter_task | 1kHz 串口打印 14 通道 |
| LedTask | Low | 128 | led_task | 流水灯 |
| BuzzerTask | Low | 128 | buzzer_task | 提示音 |

在 CubeMX 里加任务的时候，Code Generation Option 要选 `As external`，函数体留在 `applications/`，
CubeMX 只生成 extern 声明和创建语句（`Src/freertos.c` 里的 `osThreadDef` / `osThreadCreate`）。

## 硬件接口

C 板外壳丝印和 CubeMX 里的外设名**是错位的**，接线前先看这张表：

| 板上接口 | 实际外设 | 本工程用途 | 备注 |
|---|---|---|---|
| 3-pin UART，丝印 `UART1` | USART6 | **GY-56 测距** | Pin1 GND / Pin2 TXD(PG14) / Pin3 RXD(PG9)；这个口不引出 5V，模块 5V 从降压板取。9600 8N1 + RX DMA（DMA2_Stream2） |
| 4-pin UART，丝印 `UART2` | USART1 | SerialPlot | Pin1 RXD(PB7) / Pin2 TXD(PA9) / Pin3 GND / Pin4 5V |
| DBUS 口（与 PWM 共用） | USART3 | DT7 接收机 | 100000bps / 偶校验，经反相电路 |
| CAN1（2-pin，黑 CANL / 红 CANH） | CAN1 | **折叠 + 俯仰** | 折叠 GM6020(id1)、俯仰 C610+RM2006(**id4**) |
| CAN2（4-pin，与裁判系统 4-pin 跳线共用总线） | CAN2 | **偏航** | 偏航 GM6020(id2) |

## 云台控制（框架，`applications/gimbal_task.cpp`）

遥控映射（云台侧；两块板各读各的 DT7，互不干涉）：

| 输入 | 作用 |
|---|---|
| 右拨杆下档 | **云台全部电机失能**（最高优先级） |
| 右摇杆左右 `ch_rh` | 偏航角速度给定，满推 `YAW_RATE_MAX = 1.5 rad/s`（约 86 °/s） |
| 左摇杆上下 `ch_lv` | 俯仰角速度给定（增量式），满推 `PITCH_RATE_MAX = 0.8 rad/s`（约 46 °/s） |
| 左摇杆左右 / 拨轮 | 暂未分配，留给 4.4.8 目标轨迹跟踪 |

- **失能三层**（与底盘工程同一套优先级）：① 右拨杆下档 → ② 遥控器失联（`remote.is_alive()`）→ ③ 偏航/俯仰电机反馈中断（`is_alive()`）。任一层成立：清 PID 积分 + 三个电机都 `cmd(0)` + **立刻把 0 发上总线**，并且只有这一条路径能决定输出。
- **俯仰为什么用位置环**：俯仰轴有重力负载，速度环松手会掉；所以目标角由摇杆增量积分得到，再对 IMU 角度闭环，并限幅在 `PITCH_MIN_RAD ~ PITCH_MAX_RAD`（**±20°**；事故后从 ±30° 收到 ±20°，保护打印件）。
- **偏航为什么用速度环**：不需要绝对零位，用电机自身编码器速度反馈，不依赖 IMU 安装方向；`imu.yaw` 只送波形做参考。
- **限幅（2026/10/21 事故后一律收紧）**：`YAW_MAX_TORQUE = 0.3 N·m`、`PITCH_MAX_TORQUE = 0.3 N·m`、`FOLD_MAX_TORQUE = 1.2 N·m`；出力变化率另有斜坡 `YAW_OUT_SLEW = 8`、`PITCH_OUT_SLEW = 3`、`FOLD_TORQUE_SLEW = 8`（N·m/s）；PID 的总输出与积分分项各自也有上限（`max_out` / `max_iout`，两者取等防止 windup）。
  GM6020 电流模式的转矩上限 = 0.741×I_max×3（手册），24 V 供电时别越过这条线。
- **`GIMBAL_ENABLE`（`gimbal_task.hpp:19`）**：2026/10/21 起默认 `true` —— CAN 与三个电机都已在台架上实测通。改成 `false` 就完全不 `config/start` CAN、不发任何指令，只跑测距 / LED / 蜂鸣器 / 波形；改接线或第一次上电时建议先关掉它单验外设。
- **方向与符号**（2026/10/21 台架实测）：从上看**顺时针为负**、抬头("仰")时 `imu.pitch` 变小。
  所以 `YAW_SIGN = -1`（右推 = 顺时针 = 负速率）、俯仰统一用"**仰为正**"约定：反馈 `pitch_fb_ = PITCH_FB_SIGN × imu.pitch`（`PITCH_FB_SIGN = -1`），
  目标角与波形 8/9 号都用这个约定；输出方向由 `PITCH_OUT_SIGN`（默认 `+1`）标定 —— 若闭环往一边跑就改它，**别改反馈符号**。

CAN 帧（每条总线上同一帧最多 4 个电机，同帧写完再 `send`）：

| 帧 ID | 类型 | 内容 | 本工程用的电机 |
|---|---|---|---|
| `0x1FE` | 电流模式 | id1~4 各 12 bit，±2000 ↔ ±(0.741×I_max×3) N·m | 折叠 id1（CAN1）、偏航 id2（CAN2） |
| `0x200` | 电流/转矩模式 | id1~4 各 2 字节，M2006 ±10000 ↔ ±10 A | 俯仰 id4 |

反馈：GM6020 为 `0x205` / `0x206`（= id + 0x204；本工程**偏航 0x206、折叠 0x205**），C610 为 `0x204`（= id + 0x200，本工程俯仰 id4）；**两条总线都收**，回调按 `hcan1`/`hcan2` 分派；
`RM_Motor::angle / speed / torque / temperature` 由回调按 `rx_id` 分派到对应电机对象。

## 4.4.7 云台自动折叠

`applications/fold_task.cpp` + `fold_task.hpp`。规则书要求"独立任务 + 阈值/回差/状态判断 + 限位/超时/异常退出保护"，
框架里都有了：

| 要素 | 实现 |
|---|---|
| 独立任务 | `FoldTask`（10 ms）→ `fold_update()` |
| 阈值 + 回差 | 距离 < `FOLD_TRIGGER_CM`(**80 cm**，定稿值) 收折；> `FOLD_RELEASE_CM(115)` 或“前方没东西了”(超量程) 才展开 |
| 去抖 | 条件连续满足 `FOLD_DEBOUNCE_MS(300)` 才切状态 |
| 时序（按车速推算） | 折叠 0.8 rad/s、1.5 rad/s² ⇒ 单程 80° 约 **1.8 s**，状态机按 `FOLD_MOVE_TIME_MS = 2200` 判完成；触发点 80 cm 要求**过隧道车速 ≤ 0.35 m/s**（`fold_task.hpp` 里有 static_assert 校核） |
| 收折后保持 | `FOLD_MIN_HOLD_MS(3000)` 内不允许展开（≈1.8 m @0.6 m/s，按隧道长度改） |
| 状态机 | `IDLE / FOLDING / FOLDED / UNFOLDING / FAULT` |
| 距离无效时 | 保持当前状态不动（不因为"看不见"就乱动机构） |
| 异常保护 | 动作超过 `FOLD_MOVE_TIMEOUT_MS(3000)` → `FAULT` 并停机（需重新上电） |
| 输出 | `fold_target_rad()` → `gimbal_task` 的折叠电机位置环（目标角先过速度/加速度规划） |
| 无指令时 | **不发 0 转矩**：上电后第一次拿到折叠反馈时锁存当时角度，之后闭环顶住；下档失能（`disable_all()`）时不保持，下次使能重新锁存 |
| 上电零位 | **上电第一次收到折叠/偏航反馈时的角度 = 0 rad**（`fold_angle()` / `yaw_angle()`）。`FOLD_STOW_RAD` / `FOLD_DEPLOY_RAD`、偏航软限位（机械 ±60°、软限位 **±45°**）、波形全是相对这个零位；只在每次上电锁一次，失能/重新使能不重锁 |
| 收/展位置 | **上电姿态 = 展开位置（0 rad，折叠杆垂直底座）**；折叠 = 反向转 **80°**（`FOLD_STOW_DEG = 80`，`FOLD_STOW_RAD = FOLD_STOW_DIR × 80°` ≈ ∓1.396 rad）。逆时针转时 `fold_angle()` 变大还是变小取决于安装：反了只改 `FOLD_CCW_SIGN` 一处 |
| 拨杆联动 | 右拨杆**中档 → 强制展开**（`FOLD_MID_DEPLOY`，忽略测距）；上档/下档按测距自动折叠（AUTO） |
| 限位 | 非对称窗口：折叠方向 **85°**（80° + 5° 余量）、展开方向 **10°** 余量（`FOLD_LIMIT_LO_RAD` / `FOLD_LIMIT_HI_RAD`）：目标角与规划点都夹在里面 |
| 抗重力 | `fold_pid` 的 **Ki 积分**扛重力（`Ki = 8.0`、`max_iout = 1.2`）；常值前馈 `FOLD_GRAVITY_FF_NM` **默认 0**（方向要先验证：符号反了会变成压着往下，台上 FF=1.5 仍下沉就是符号问题）。`FOLD_MAX_TORQUE = 1.2`（驱动层封顶 2.223 N·m，事故后从 2.2 收到 1.2）。建模估算保持力矩：展开姿态约 **0.2 N·m**，转到臂水平最大约 **1.0 N·m** |

**台架待做**：① 确认折叠方向与机械限位（展开 0° / 折叠反向 80° 已按现场确认值写入；方向反了就把 `FOLD_STOW_DIR` 改成 +1.0f）；
② 把 `FOLDING / UNFOLDING` 的"到位判据"从"测距条件变化"改成"角度误差 + 稳定时间"（折叠动作会带着测距模块动）；
③ `FAULT` 目前只能重新上电退出，计划加"拨杆保持下档 3 s 复位"；④ 用隧道实测长度整定 `FOLD_MIN_HOLD_MS`、实测触发距离整定 `FOLD_TRIGGER_CM` / `FOLD_RELEASE_CM`。

## 4.4.8 目标轨迹跟踪（骨架已接入，默认关）

组委会给了参考轨迹（`D:\university\RM\stage_2\trajectory_task`：1 kHz 采样 / 680 ms 整周期），
`trajectory_task.cpp/.hpp` 已拷进 `applications/` 并加进 `CMakeLists.txt`。

- **触发方式（两个条件同时满足）**：① `gimbal_task.cpp` 的编译期开关 `TRAJ_ENABLE = true`；② **右拨杆打到上档**。
  上档瞬间开始按组委会轨迹（680 ms 一周期）循环跑，离开上档回中档立刻退回手动摇杆控制，下次上档从轨迹起点重新开始；
  右拨杆下档 / 遥控失联 / 电机反馈中断这三条失能照旧最高优先级（下档会连相位一起复位）。
  右拨杆下档 / 遥控失联 / 电机反馈中断这三条失能照旧最高优先级。
- **跟踪方式**（有意做成前馈 + 外环，而不是等误差出来再 PI 追）：
  - 偏航：角度外环 P（`TRAJ_YAW_KP`）→ 角速度给定 = Kp·(θ_ref − θ_imu) + k_v·ω_ref → 原有速率环；
  - 俯仰：轨迹角 + `TRAJ_PITCH_LEAD_S`（0.03 s）超前 → 位置环 → 再叠一个 `TRAJ_PITCH_ACC_FF`×加速度的电压前馈。
- **轨迹指标**：pitch ±30.00°（峰值 8.31 rad/s、69 rad/s²）、yaw ±22.50°（峰值 7.15 rad/s、107 rad/s²）。
  这张表在 `TRAJ_ENABLE=false` 时会被链接器整段丢掉，不占 FLASH。
- **打开前必须做**：① 机械干涉确认（指导文档 §2.3 的包围盒粗算提示 + 方向约 +23° 会压到底板，
  要用 SW 实体干涉复核）；② 机械限位实测；③ `PITCH_MAX_VOLT` 从 6 V 往大试。
- 没有另起 FreeRTOS 任务：控制环本身就是 1 kHz，直接调 `trajectory_sample_at_ms()`，
  少一层调度抖动（组委会那个任务函数照样编译进镜像，需要时可以切过去）。

## SerialPlot 14 通道

| 通道 | 含义 |
|---|---|
| 0 / 1 | 测距（3 点中值，cm）/ 测距状态码（0 正常 · 1 无帧 · 2 超时 · 3 帧错 · 4 无效值或超量程） |
| 2 / 3 / 4 | IMU yaw / pitch / roll（rad）；`YAW_SPEED_PLOT = true`（默认）时 4 号改显**偏航实际角速度** rad/s |
| 5 / 6 / 7 | 偏航目标角速度 / 折叠目标角（已限速规划, rad）/ 折叠实际角（rad） |
| 8 / 9 / 10 | 俯仰目标角（仰为正）/ 实际角（仰为正，`pitch_fb_`）/ 控制量（N·m） |
| 11 | **失能原因码**（调试）：0 正常 / 1 下档 / 2 遥控失联 / 3 偏航反馈丢 / 4 俯仰反馈丢 / **5 姿态或跟随异常（联锁）** |
| 12 | **反馈帧位图**（调试）：bit0~bit10 对应 `0x201~0x20B`，例如 24 = 0x204(俯仰) + 0x205(折叠) 都在；48 = 再加 0x206(偏航) |
| 13 | **右开关档位**（调试）：0 下 / 1 中 / 2 上 |

> 11/12/13 是**调试**通道：11 用来看三层失能卡在哪一层，12 用来核对电机 ID/接线，13 用来确认拨杆档位。
> 6/7 号原为偏航速度/偏航输出，偏航轴暂停期间改看折叠（目标角应是一条斜坡，不是阶跃）。
> 折叠速度由 `FOLD_SPEED_MAX`（默认 **1.2 rad/s ≈ 69°/s**）与 `FOLD_ACC_MAX`（**3.0 rad/s²**）限制。
> 触发距离 80 cm 是按用户定稿；配 `FOLD_MOVE_TIME_S = 2.2`，要求过隧道车速 ≤ `CHASSIS_SPEED_MPS = 0.35 m/s`。车速更快就折不完（编译期 static_assert 会报）。

> 右拨杆**上档**（且 `TRAJ_ENABLE = true`）时：5 号通道换成**轨迹偏航角**、8 号通道换成**轨迹俯仰角**（rad）；回中档变回摇杆目标。
>
> 上档进入时参考角**从当前姿态连续过渡**（`TRAJ_ANCHOR_ON_ENTER`：参考 = 上档时姿态 + 幅度缩放 ×(轨迹角 − 轨迹起点角)），不再一进去就跳到轨迹起点（yaw +18° / pitch −30°）。幅度按 `TRAJ_AMP_SCALE`（默认 **0.3**）缩放，俯仰参考另有 `TRAJ_REF_SLEW = 2.0f` rad/s 的变化率上限。跑顺了就把 `TRAJ_AMP_SCALE` 往 0.5 → 0.8 → 1.0 加。

**安全联锁（2026/10/21 事故后加）**

- `ATTITUDE_GUARD_ENABLE = true`：|横滚| > 15°，或俯仰跟随误差 > 20° 且已在出力，持续 50 ms → 判定机构异常，**立即失能（原因码 5）**，所有输出归零。原则是“撒手”，不用大转矩去救。
- `TRAJ_REQUIRE_FOLD_DEPLOY = true`：折叠不在展开位（`FOLD_DEPLOY_RAD` ±5°）时**不允许进轨迹跟踪**，防止上段已经倒下、轨迹还在纠偏把结构扯坏。
- `PITCH_MAX_TORQUE` 从 1.8 收到 **0.5 N·m**：PLA 打印件扛不住大转矩；机构换成尼龙/PA-CF 或加金属轴套并手托验证后，再按 0.5 → 0.8 → 1.2 逐步加。


帧长 = 3 + 14×4 = **59 字节**，`size = 0x38`。

## sp_middleware 说明

本工程把 `sp_middleware`（同济 SuperPower 中间件，上游 `TongjiSuperPower/sp_middleware`，提交 `eddf9a35`）
以**源码形式**随工程携带，不再做成 git 子模块（`.git` / `.gitmodules` 已移除）。
其中未修改任何文件（与上游 `eddf9a35` 一致）。SerialPlot 的 DMA 门控是**应用层**改动，在
`applications/plotter_task.cpp` 里。若以后要改回子模块形式，注意"库改动不会随主仓库提交"的坑。

## 红外测距（规则书 4.4.2.1）

GY-56（VL53L0X）出厂为 UART 9600 8N1、高精度模式（200 ms）连续输出。帧格式（**长度可变**）：

```
5A 5A 15 L | DH DL | mode | temp | [其余数据] | sum     共 5+L 字节（L = 数据量，手册 8~13 字节）
校验和 = 前 (4+L) 字节之和的低 8 位      距离 = (DH<<8)|DL，单位 cm
mode: 0 长距离 2m/35ms · 1 快速 22ms · 2 高精度 200ms(出厂默认) · 3 一般 35ms
```

`applications/tof_task.cpp` 的处理（**不是**固定 9 字节）：

| 处理 | 做法 |
|---|---|
| 帧长 | 读 `Byte3` 的数据量算 `5+L`；缓冲 24 字节，一次回调里循环取完所有完整帧 |
| 校验 | 校验前 `4+L` 字节之和；错则丢弃（`tof_status = 3`，`tof_bad_sum++`） |
| 噪声重同步 | 帧头/帧类型/长度非法时逐字节向后挪**重新找帧头**，一个脏字节不会丢掉后面所有帧 |
| 无效值 / 超量程 | `d == 0` 或 `d > TOF_RANGE_CM(120)` → 不参与决策（`tof_status = 4`，`tof_out_range++`） |
| 中值滤波 | 每帧推入 3 点窗口，输出的 `tof_distance_cm` 是窗口**中位数** |
| 通信超时 | 距上次有效帧 > `TOF_TIMEOUT_MS(500)` → `tof_valid = false`、`tof_status = 2`、`tof_timeouts++`；从未收到过帧是 1 |
| DMA 半满 | `uart_task.cpp` 里显式忽略 `HAL_UART_RXEVENT_HT`（HAL 不会关掉半满中断，半满时只有半帧） |
| 配置命令 | 上电等 `TOF_WARMUP_MS(1000)` 让模块完成自检，再发"一般模式 `A5 54 03 FC` + 连续输出 `A5 65 01 0B`"，**故意不保存**；连续 2 s 收不到有效帧就重发 |

对外接口：`tof_distance_cm`（中值后，`<0` 无效）、`tof_raw_cm`、`tof_valid`、`tof_status`、`tof_mode`、`tof_temp_c`、
`tof_last_ms` 和 5 个计数（`tof_frames / tof_bad_sum / tof_out_range / tof_timeouts / tof_errors`）。
上层（4.4.7 折叠逻辑）统一调 `bool tof_get_distance(float* cm, uint32_t now_ms)`，**不要看裸距离**。

## 注意事项

安全方面：

1. 动云台电机之前，先确认 DT7 右拨杆下档能让三个电机可靠失能
2. 烧录、接线、拆装、人工转机构之前，把动力电源断掉
3. 改完代码第一次测试，把机构固定住或者让人扶稳，输出限幅先给低一点
4. 出现失控、异响、异味、过热、冒烟、线束被拉扯、结构松动，立刻失能断电

## 已确认 / 待确认

已确认：

- **红外测距代码侧已完成**（动态帧长 + 中值 + 状态码 + 配置命令），`TOF_ENABLE = true`
- **两块 C 板的职责划分**：底盘板只做运动与功率，其余（测距 / 折叠 / 基本功能）在云台板
- 云台电机：偏航 **id2**（CAN2）/ 折叠 **id1**（CAN1），都是 GM6020 电流模式、帧 `0x1FE`；俯仰 **id4**（CAN1，C610 + RM2006，0x200）

待确认（现场）：

1. 云台电机实际 ID、转向与机械限位（→ `YAW_SIGN` / `PITCH_SIGN` / `PITCH_MIN_RAD` / `PITCH_MAX_RAD`）
2. 折叠机构的真实减速比与行程（→ `fold_motor` 的减速比、`FOLD_STOW_RAD` / `FOLD_DEPLOY_RAD`）
3. 折叠触发/释放阈值（→ `FOLD_TRIGGER_CM` / `FOLD_RELEASE_CM`，需按隧道实测）
4. 偏航/俯仰 PID 与限幅整定（先离地、低限幅）

## 参考

- RoboMaster 开发板 C 型用户手册、原理图
- RoboMaster GM6020 直流无刷电机使用说明、M2006 + C610 说明
- RoboMaster 机器人专用遥控器（接收机）用户手册
- GY-56 红外测距模块使用说明
- `sp_middleware` 各模块的 readme
- 底盘工程 `..\files\stage_2`（4.4.2.2 / 4.4.2.3 与功率计解析）
