# cboard_gimbal（云台 C 板）

RoboMaster 开发板 C 型（STM32F407IGHx）上的**云台**嵌入式工程。
当前进度：**第二阶段 · 云台**——云台电机控制框架 + 红外测距（4.4.2.1）已完成；4.4.7 自动折叠只出框架；4.4.8 轨迹跟踪待做。

## 两块 C 板的分工

| 工程 | 目录 | 职责（规则书） | 遥控器 | 管的电机 |
|---|---|---|---|---|
| 底盘 C 板 | `..\files\stage_2` | 4.4.2.2 遥控器与底盘控制、4.4.2.3 功率控制、功率计 | 自己读 DT7 | 4×RM3508 + C620（CAN1） |
| **云台 C 板（本工程）** | `..\files\stage_2_gimbal` | 4.4.2.1 红外测距 + C 板基本功能（LED / 蜂鸣器）、4.4.7 云台自动折叠、4.4.8 目标轨迹跟踪 | 自己读 DT7 | 偏航 GM6020、俯仰 GM6020、折叠 C610 + RM2006（CAN1） |

两块板**各自独立**：各自读自己的 DT7 接收机、各自驱动自己的电机，不互相转发遥控指令，
也不共享串口数据。本工程**不含**底盘运动解算、功率控制与功率计解析（那些在底盘工程里）。

## 功能

- 蜂鸣器提示音、RGB 流水灯（C 板基本功能）
- IMU(BMI088) 采集 + Mahony 姿态解算，三轴姿态角经 SerialPlot 输出（USART1 / 921600 / 帧头 `AA BB`）
- DT7 遥控器通讯
- **GY-56 红外测距读取（4.4.2.1）**：动态帧长（8~13 字节）、3 点中值滤波、帧头/校验/无效值/超量程/超时分别计数并给状态码，对外提供 `tof_get_distance()` 供 4.4.7 折叠逻辑调用
- **云台电机控制框架**：偏航（GM6020 电压模式，角速度环）+ 俯仰（GM6020 电压模式，增量式位置环）+ 折叠机构（C610 + RM2006 位置环）接口
- **4.4.7 折叠决策骨架**：独立任务、阈值 + 回差 + 去抖 + 动作超时保护（`FOLD_ENABLE=false`，未接线）

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
| CAN1（2-pin，黑 CANL / 红 CANH） | CAN1 | **云台电机** | 偏航 GM6020(id1)、俯仰 GM6020(id2)、折叠 C610+RM2006(id3) |
| CAN2（4-pin） | CAN2 | 预留 | 本工程不用；功率计在底盘工程那条总线上 |

## 云台控制（框架，`applications/gimbal_task.cpp`）

遥控映射（云台侧；两块板各读各的 DT7，互不干涉）：

| 输入 | 作用 |
|---|---|
| 右拨杆下档 | **云台全部电机失能**（最高优先级） |
| 右摇杆左右 `ch_rh` | 偏航角速度给定，满推 `YAW_RATE_MAX = 6.0 rad/s` |
| 左摇杆上下 `ch_lv` | 俯仰角速度给定（增量式），满推 `PITCH_RATE_MAX = 1.5 rad/s` |
| 左摇杆左右 / 拨轮 | 暂未分配，留给 4.4.8 目标轨迹跟踪 |

- **失能三层**（与底盘工程同一套优先级）：① 右拨杆下档 → ② 遥控器失联（`remote.is_alive()`）→ ③ 偏航/俯仰电机反馈中断（`is_alive()`）。任一层成立：清 PID 积分 + 三个电机都 `cmd(0)` + **立刻把 0 发上总线**，并且只有这一条路径能决定输出。
- **俯仰为什么用位置环**：俯仰轴有重力负载，速度环松手会掉；所以目标角由摇杆增量积分得到，再对 IMU 角度闭环，并限幅在 `PITCH_MIN_RAD ~ PITCH_MAX_RAD`（先给 ±30°，按实际机构改）。
- **偏航为什么用速度环**：不需要绝对零位，用电机自身编码器速度反馈，不依赖 IMU 安装方向；`imu.yaw` 只送波形做参考。
- **限幅**：`YAW_MAX_VOLT = 24 V`、`PITCH_MAX_VOLT = 12 V`（先给一半）、`FOLD_MAX_TORQUE = 1 N·m`，PID 的总输出与积分分项各自也有上限（`max_out` / `max_iout`）。
- **`GIMBAL_ENABLE = false`（默认）**：不 `config/start` CAN、不发任何指令，只跑测距 / LED / 蜂鸣器 / 波形 —— 台架上可以安全地先把 4.4.2.1 调完。接线确认转向与限位后再改成 `true`。
- **方向标定**：`YAW_SIGN` / `PITCH_SIGN`（默认 `+1`）。上机后若"推右车/云台往左"之类反了，就把对应符号取反（与底盘工程 `FWD_DIR/SIDE_DIR/ROT_DIR` 同一套路）。

CAN 帧（每条总线上同一帧最多 4 个电机，同帧写完再 `send`）：

| 帧 ID | 类型 | 内容 | 本工程用的电机 |
|---|---|---|---|
| `0x1FF` | 电压模式 | id1~4 各 2 字节，±25000 ↔ ±24 V | 偏航 id1、俯仰 id2 |
| `0x200` | 电流/转矩模式 | id1~4 各 2 字节，M2006 ±10000 ↔ ±10 A | 折叠 id3 |

反馈：GM6020 为 `0x205` / `0x206`（= id + 0x204），C610 为 `0x203`（= id + 0x200）；
`RM_Motor::angle / speed / torque / temperature` 由 `HAL_CAN_RxFifo0MsgPendingCallback` 分发。

## 4.4.7 云台自动折叠（框架）

`applications/fold_task.cpp` + `fold_task.hpp`。规则书要求"独立任务 + 阈值/回差/状态判断 + 限位/超时/异常退出保护"，
框架里都有了：

| 要素 | 实现 |
|---|---|
| 独立任务 | `FoldTask`（10 ms）→ `fold_update()` |
| 阈值 + 回差 | 距离 < `FOLD_TRIGGER_CM(60)` 收折；> `FOLD_RELEASE_CM(75)` 才展开（回差 15 cm） |
| 去抖 | 条件连续满足 `FOLD_DEBOUNCE_MS(300)` 才切状态 |
| 状态机 | `IDLE / FOLDING / FOLDED / UNFOLDING / FAULT` |
| 距离无效时 | 保持当前状态不动（不因为"看不见"就乱动机构） |
| 异常保护 | 动作超过 `FOLD_MOVE_TIMEOUT_MS(2000)` → `FAULT` 并停机（需重新上电） |
| 输出 | `fold_target_rad()` → `gimbal_task` 的折叠电机位置环；`FOLD_ENABLE=false` 时永远返回 false（发 0） |

**接线后待做**：① 量出收折/展开的真实电机角，改 `FOLD_STOW_RAD` / `FOLD_DEPLOY_RAD`（现在是占位值）；
② 接限位开关，把 `FOLDING / UNFOLDING` 的"到位判据"改成限位 + 位置双判据；③ 用隧道实测距离整定 `FOLD_TRIGGER_CM` / `FOLD_RELEASE_CM`；④ 把 `FOLD_ENABLE` 改成 `true`，先手动给低限幅试动作。

## 4.4.8 目标轨迹跟踪（待做）

本工程已留好姿态接口 `gimbal_yaw_rad()` / `gimbal_pitch_rad()`（`gimbal_task.hpp`）。
要做的是：给云台加"目标角/目标角速度"外部给定（把现在的摇杆增量给定抽象成一层），
再接视觉/裁判系统的目标信息。规则书的限位、超时、异常退出保护可直接复用 4.4.7 的写法。

## SerialPlot 14 通道

| 通道 | 含义 |
|---|---|
| 0 / 1 | 测距（3 点中值，cm）/ 测距状态码（0 正常 · 1 无帧 · 2 超时 · 3 帧错 · 4 无效值或超量程） |
| 2 / 3 / 4 | IMU yaw / pitch / roll（rad） |
| 5 / 6 / 7 | 偏航目标角速度 / 实际角速度（电机）/ 控制量（V） |
| 8 / 9 / 10 | 俯仰目标角 / 实际角（IMU）/ 控制量（V） |
| 11 | 折叠状态码（0 IDLE · 1 FOLDING · 2 FOLDED · 3 UNFOLDING · 4 FAULT） |
| 12 / 13 | 折叠电机目标角（rad）/ 是否被驱动（0/1） |

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
- 云台电机规划：偏航 id1 / 俯仰 id2（GM6020 电压模式，0x1FF）、折叠 id3（C610 + RM2006，0x200）

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
