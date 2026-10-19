# cboard_gimbal（云台 C 板）

RoboMaster 开发板 C 型（STM32F407IGHx）上的**云台**嵌入式工程。
当前进度：**第二阶段 · 云台**——双 CAN 接收诊断 + 单电机 20° 相对往返测试；红外测距（4.4.2.1）保留；4.4.7 折叠只保留决策骨架；4.4.8 轨迹跟踪待做。云台**不接 DT7/遥控器**，正常 yaw/pitch 遥控控制已移除。

> **默认构建 `FOLD_TEST_ENABLE=0`：烧录/复位/上电不触发 20° 动作，只进入双 CAN 只接收诊断。`gm6020_fold_test.*`、参数、保护、返回修复和构建源清单全部保留；未来把 `FOLD_TEST_ENABLE` 改为 1 并重新构建，才会恢复“上电收到有效反馈 500 ms 后自动执行一次”。**

## 两块 C 板的分工

| 工程 | 目录 | 职责（规则书） | 管的电机 |
|---|---|---|---|
| 底盘 C 板 | `..\files\stage_2` | 4.4.2.2 遥控器与底盘控制、4.4.2.3 功率控制、功率计 | 4×RM3508 + C620（CAN1） |
| **云台 C 板（本工程）** | `..\files\stage_2_gimbal` | 4.4.2.1 红外测距 + C 板基本功能（LED / 蜂鸣器）、4.4.7 云台自动折叠、4.4.8 目标轨迹跟踪 | GM6020×2、C610 + RM2006 |

两块板**各自驱动自己的电机，不互相转发遥控指令，也不共享串口数据**（云台侧已无遥控接收机）。
本工程**不含**底盘运动解算、功率控制与功率计解析（那些在底盘工程里）。

## 功能

- 蜂鸣器提示音、RGB 流水灯（C 板基本功能）
- IMU(BMI088) 采集 + Mahony 姿态解算，三轴姿态角经 SerialPlot 输出（USART1 / 921600 / 帧头 `AA BB`）
- **双 CAN 接收诊断**：CAN1/CAN2 候选反馈、ID、计数、帧龄；不发送控制帧
- **单电机 20° 相对往返测试（默认关闭，代码保留）**：`FOLD_TEST_ENABLE=1` 并重新构建后才控制 CAN1 GM6020 ID1（0x205）；默认只做双 CAN 接收
- **GY-56 红外测距读取（4.4.2.1）**：动态帧长（8~13 字节）、3 点中值滤波、帧头/校验/无效值/超量程/超时分别计数并给状态码，对外提供 `tof_get_distance()` 供 4.4.7 折叠逻辑调用
- **4.4.7 折叠决策骨架**：独立任务、阈值 + 回差 + 去抖 + 动作超时保护（`FOLD_ENABLE=false`，未接线；当前无正常折叠执行器控制）

## 任务

| 任务 | 优先级 | 栈(字) | 入口 | 干什么 |
|---|---|---|---|---|
| defaultTask | Normal | 128 | StartDefaultTask | USB 初始化 |
| ImuTask | AboveNormal | 512 | imu_task | 1kHz 采 IMU 并解算 |
| UartTask | Normal | 256 | uart_task | GY-56 串口回调转发（USART6）；不再使用 DT7/DBus |
| TofTask | Normal | 256 | tof_task | 上电等 1 s 自检 → 发配置命令 → 10 ms 巡检 GY-56（动态帧长 + 中值 + 超时判无效） |
| **GimbalTask** | AboveNormal | 256 | gimbal_task | 默认双 CAN 接收诊断；20° 测试代码保留但 `FOLD_TEST_ENABLE=0`，不发送控制帧 |
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
| CAN1（2-pin，黑 CANL / 红 CANH） | CAN1 | **云台电机** | 实测：GM6020 ID1（0x205）、C610 ID4（0x204） |
| CAN2（4-pin） | CAN2 | **底部 6020** | 实测：GM6020 ID2（0x206） |

## 控制方式（无遥控）

云台板不再接收 DT7/DBus，正常 yaw/pitch 遥控控制已移除。**当前默认只运行双 CAN 接收诊断，上电不动作**；只有显式把 `FOLD_TEST_ENABLE` 改为 1 并重新构建，才进入单电机 20° 往返测试。

CAN 帧（每条总线上同一帧最多 4 个电机，同帧写完再 `send`）：

| 帧 ID | 类型 | 内容 | 当前用途 |
|---|---|---|---|
| `0x1FE` | 电流/转矩模式 | id1~4 各 2 字节，±16384 ↔ ±3 A ↔ ±2.223 N·m | 测试只写 ID1 第 1 槽；其余槽清零；CAN2 发全零 |
| `0x200` | 电流/转矩模式 | id1~4 各 2 字节，M2006 ±10000 ↔ ±10 A | 测试发全零给 C610 |

反馈：GM6020 为 `id + 0x204`（实测 0x205/0x206），C610 为 `id + 0x200`（实测 0x204）；由 `can_diag` 按实际 `hcan` 分发，测试通过 frame hook 取 0x205。

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
| 输出 | `fold_target_rad()` 只保留给后续执行器；当前 `FOLD_ENABLE=false`，且无正常云台折叠位置环，`gimbal_task` 不驱动折叠电机 |

**接线后待做**：① 量出收折/展开的真实电机角，改 `FOLD_STOW_RAD` / `FOLD_DEPLOY_RAD`（现在是占位值）；
② 接限位开关，把 `FOLDING / UNFOLDING` 的"到位判据"改成限位 + 位置双判据；③ 用隧道实测距离整定 `FOLD_TRIGGER_CM` / `FOLD_RELEASE_CM`；④ 把 `FOLD_ENABLE` 改成 `true`，先手动给低限幅试动作。

## 4.4.8 目标轨迹跟踪（待做）

本工程已留好姿态接口 `gimbal_yaw_rad()` / `gimbal_pitch_rad()`（`gimbal_task.hpp`）。
要做的是：给云台加“目标角/目标角速度”外部给定（当前无遥控输入，需另接视觉/裁判/其他板卡），
再接目标信息。规则书的限位、超时、异常退出保护可参考 4.4.7 的写法。

## SerialPlot 14 通道

正常遥控控制已移除；当前 14 通道映射见下面的「单电机 20° 相对往返测试」或「双 CAN 接收诊断模式」。帧长仍为 3 + 14×4 = **59 字节**，`size = 0x38`。

## 单电机 20° 相对往返测试（保留，默认 `FOLD_TEST_ENABLE=0`）

默认构建**不进入**本测试；把 `applications/gm6020_fold_test.hpp` 的 `FOLD_TEST_ENABLE` 改为 1 并重新构建后才会进入（高于纯 CAN 诊断）：**只控制 CAN1 上 GM6020 ID1（反馈 0x205）**，电流/转矩模式 `0x1FE` 第 1 槽。其他 6020、M2006/C610 禁止非零输出；正常云台控制/原折叠状态机不执行，不使用旧 `FOLD_MOTOR_ID=3`。

实测接收核查：CAN1 反馈 `0x205`（GM6020 ID1）与 `0x204`（C610 候选 ID4），CAN2 反馈 `0x206`（底部 GM6020 ID2）。本测试只使用 `0x205`，其余电机只发零帧或零槽。

协议与单位：

- `sp::RM_Motors::GM6020` 是**电流/转矩模式**：ID1~4 控制帧 `0x1FE`，ID5~7 为 `0x2FE`；`cmd()` 参数单位是 **N·m**，由库换算成原始电流值。
- 库量程：`±16384 raw ↔ ±3 A ↔ ±2.223 N·m`（`GM6020_MAX_TORQUE = 0.741×3`，`GM6020_MAX_RAW = 16384`）。
- 本测试 `kMaxTorqueNm = 0.25 N·m`：`0.25 / 0.741 ≈ 0.337 A`，`0.25 × 16384 / 2.223 ≈ 1842 raw`，仍在 `±16384` 内。
- CAN2 只发 `0x1FE` 全零，所有测试路径（启动/运行/DONE/ABORT/配置异常）都不再发 `0x1FF`；CAN1 `0x200` 全零 C610 帧保留。
- 发送侧检查 `HAL_CAN_AddTxMessage()` 返回值；失败进入 CAN_ERROR 并零输出。返回值只证明入队成功，不证明总线/电机实际收到。
- 橙灯常亮表示“电流控制模式下收到电压指令”；本修复统一改为电流模式 `0x1FE`，但实际灯态需烧录后确认。

往/返/停留统一行程范围：`[start - 0.15 rad, start + 20° + 0.15 rad]`，覆盖 0° 起点和 +20° 端点；超出仍中止。

- 方向：用户接受两个方向任选，当前取**编码器正方向 +20°**；该符号只表示编码器角度增加，**不声称对应物理顺/逆时针，也不等价于从输出轴端看的顺/逆时针**。上电前必须人工确认 20° 机械/线束空间并准备断电。
- 启动：无人工触发。**启用后**上电，CAN 两路启动、收到有效 0x205 并稳定 **500 ms**，自动开始一次；完成/中止后不重试，需重新上电。
- 动作：启动时捕获当前有效编码器位置为起点 → 缓慢限速到相对起点 +20° → 位置/速度稳定持续 200 ms → 停留 300 ms → 缓慢返回起点 → 稳定 200 ms → 完成。不循环。
- 完成/中止后**零输出**；零输出不保证重力机构保持位置，不能把本测试当作完整折叠安全验收。
- 0x205 反馈超时、角度/速度非有限值、总超时、行程异常都会立即零输出并中止；中止后不自动强行复原。云台无遥控急停，异常时断动力电源。

保守初值（无负载/实机整定证据，必须台架确认）：

| 参数 | 值 | 说明 |
|---|---:|---|
| 相对行程 | 20°（0.34906585 rad） | 从启动起点算 |
| 最大输出转矩 | 0.25 N·m | ≈0.337 A，≈+1842 raw；远低于 ±3 A / ±2.223 N·m 满量程 |
| Kp / Kd | 1.2 N·m/rad / 0.06 N·m/(rad/s) | 位置 P + 速度阻尼 |
| 参考速度 / 加速度 | 0.15 rad/s / 0.2 rad/s² | 缓慢移动 |
| 位置 / 速度容差 | 0.03 rad / 0.05 rad/s | 到位判据 |
| 稳定 / 停留 | 200 ms / 300 ms | 到位后短暂停留 |
| 总超时 | 12 s | 超时中止 |
| 行程异常 | 超出目标 0.15 rad | 中止 |

### 测试模式 SerialPlot（仍为 14 通道 / 59 字节）

帧格式不变：`AA BB + size(0x38) + 14×float`，USART1 921600。

| 通道 | 含义 |
|---:|---|
| 0 | 状态：0 CONFIG_LOCKED / 1 WAIT_FEEDBACK / 2 READY（自动等待）/ 3 MOVE_OUT / 4 HOLD / 5 MOVE_BACK / 6 DONE / 7 ABORTED / 8 CAN_ERROR |
| 1 | 相对目标角（度） |
| 2 | 相对实际角（度，相对启动起点） |
| 3 | 误差（度） |
| 4 | 速度（度/秒） |
| 5 | 输出转矩（N·m） |
| 6 | 0x205 反馈帧龄（ms；-1=从未收到） |
| 7 | 自动启动已开始 0/1 |
| 8 | 故障码：0 无 / 1 配置锁定 / 2 无反馈 / 3 反馈丢失 / 4 超时 / 5 非有限值 / 6 行程异常 / 7 CAN错误 |
| 9 | CAN1 候选在线位图 |
| 10 | CAN2 候选在线位图 |
| 11 | CAN1 最后 RX ID |
| 12 | CAN2 最后 RX ID |
| 13 | CAN1 0x205 累计有效帧数 |

位图/候选/未知 ID 的详细统计仍可在 `can_diag::buses` 调试变量查看。

## 双 CAN 接收诊断模式（默认：`CAN_DIAG_ENABLE=1`，`FOLD_TEST_ENABLE=0`）

当 `FOLD_TEST_ENABLE=0` 且 `CAN_DIAG_ENABLE=1` 时进入纯接收诊断：`gimbal_task` 执行 `can_diag` 分支，**不进入任何电机控制**；诊断模式只初始化 CAN1/CAN2 接收，**不发送任何 CAN 帧**（包括 `0x200`/`0x1FF` 零控制帧），诊断路径不创建测试电机对象、不调用 `cmd()`/`send()`，也不自动运动。

当前接线信息：三台电机为两台 GM6020 + 一台 M2006/C610；**底部 GM6020 已确认接 CAN2**。另一台 GM6020 的总线、全部拨码 ID、M2006/C610 的总线/ID 尚无最新确认；旧的 CAN1/ID1、ID2、M2006 ID3 只能作为候选，不能当实物结论。

保持默认纯诊断：确认 `FOLD_TEST_ENABLE=0` 且 `CAN_DIAG_ENABLE=1`；两开关都关时 `gimbal_task` 只空转，当前没有可切回的正常遥控云台控制。

- CAN1 滤波器 bank **0**；CAN2 滤波器 bank **14**，`SlaveStartFilterBank=14`；两路都按 1 Mbps 配置，使用 RX **FIFO0** / `CANx_RX0_IRQn`。
- `Src/can.c` 的 CAN2 MspInit 会同时使能 CAN1 时钟；`main.c` 先 `MX_CAN1_Init()` 再 `MX_CAN2_Init()`。
- 回调 `HAL_CAN_RxFifo0MsgPendingCallback` 按实际 `hcan` 分发到 CAN1/CAN2，不再用单总线入口过滤。
- 候选反馈 ID（只作候选，不自动认定型号）：
  - GM6020 候选：`0x205 ~ 0x20B`（bit0~bit6）
  - M2006/C610 候选：`0x201 ~ 0x204`（bit7~bit10）
- 有效反馈判据：**标准帧 + 数据帧 + DLC=8 + 候选 ID**，且距最近有效帧 <100 ms 才算在线；从未收到帧不算在线。
- 扩展帧、非候选标准 ID 计入“未知/异常”；候选 ID 但 IDE/RTR/DLC 不合法计入 invalid，不计在线。

### 诊断模式 SerialPlot（仍为 14 通道 / 59 字节）

帧格式不变：`AA BB + size(0x38) + 14×float`，USART1 921600，Custom Frame、First byte size、float32、Little Endian、无校验。

| 通道 | 含义 |
|---:|---|
| 0 | CAN1 原始收帧总数（所有 ID） |
| 1 | CAN2 原始收帧总数（所有 ID） |
| 2 | CAN1 最后收到的原始 ID（标准或扩展） |
| 3 | CAN2 最后收到的原始 ID |
| 4 | CAN1 候选在线位图（bit0~bit10） |
| 5 | CAN2 候选在线位图 |
| 6 | CAN1 候选累计有效帧数 |
| 7 | CAN2 候选累计有效帧数 |
| 8 | CAN1 在线候选中的最老帧龄 ms（0=无在线候选） |
| 9 | CAN2 在线候选中的最老帧龄 ms |
| 10 | CAN1 未知帧 + 候选格式错误帧计数 |
| 11 | CAN2 未知帧 + 候选格式错误帧计数 |
| 12 | 状态：0 未初始化 / 1 两路已启动 / 2 初始化或启动失败 / 3 接收错误 |
| 13 | 最近一个未知 ID（任意总线；0=无或 ID 为 0） |

位图解码：

| bit | ID | 候选 |
|---:|---:|---|
| 0~6 | 0x205~0x20B | GM6020 候选 |
| 7~10 | 0x201~0x204 | M2006/C610 候选 |

逐候选的 `valid_count`、`invalid_count`、`last_valid_ms`，以及未知 ID 小表（每路 16 槽）在调试变量中查看：

```cpp
can_diag::buses[0] / can_diag::buses[1]
can_diag::buses[b].candidates[i]
can_diag::buses[b].unknown[j]
can_diag::state
```

限制：

- 同一路上同一个 ID 的计数不能区分两台物理电机；不能凭 ID 自动认定型号。
- 14 通道给出总线级汇总；逐候选计数/帧龄用调试变量补充，不把通道含义做成滚动表。
- 诊断模式不会命令 2006/C610 或任何电机；C610 若因无命令进入自身保护，属于预期外设行为。
- 实际三台是否在线必须由烧录后的真实反馈确认；编译通过不等于通信正常。

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

1. 动云台电机之前，先确认 20° 机械/线束空间；云台无遥控急停，异常时直接断动力电源
2. 烧录、接线、拆装、人工转机构之前，把动力电源断掉
3. 改完代码第一次测试，把机构固定住或者让人扶稳，输出限幅先给低一点
4. 出现失控、异响、异味、过热、冒烟、线束被拉扯、结构松动，立刻断电

## 已确认 / 待确认

已确认：

- **红外测距代码侧已完成**（动态帧长 + 中值 + 状态码 + 配置命令），`TOF_ENABLE = true`
- **两块 C 板的职责划分**：底盘板只做运动与功率，其余（测距 / 折叠 / 基本功能）在云台板
- **云台无遥控**：DT7/DBus 接收与正常 yaw/pitch 遥控控制已移除；20°测试上电自动执行一次
- 实测接收：CAN1 `0x205`（GM6020 ID1）、`0x204`（C610 ID4）；CAN2 `0x206`（底部 GM6020 ID2）

待确认（现场）：

1. 20° 相对行程的机械/线束空间、实际转向与输出轴端顺逆关系
2. 0.25 N·m / Kp 1.2 / Kd 0.06 等初值的实机整定与到位/回零精度
3. 折叠机构真实减速比/行程、C610 ID4 映射及后续折叠执行器控制
4. 无遥控后的整机控制源（视觉/裁判/其他板卡）与完整安全验收

## 参考

- RoboMaster 开发板 C 型用户手册、原理图
- RoboMaster GM6020 直流无刷电机使用说明、M2006 + C610 说明
- GY-56 红外测距模块使用说明
- `sp_middleware` 各模块的 readme
- 底盘工程 `..\files\stage_2`（4.4.2.2 / 4.4.2.3 与功率计解析）
