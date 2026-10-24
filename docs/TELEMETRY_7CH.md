# 七通道SerialPlot配置

串口USART1（UART2），460800、8N1、无流控；发送周期10 ms。控制、官方评分仍在1 ms任务中运行。

| SerialPlot通道 | 含义 | 单位 |
|---|---|---|
| 1 | Yaw实时命中比例 | 0～1 |
| 2 | Pitch实时命中比例 | 0～1 |
| 3 | Yaw目标角 | 度 |
| 4 | Yaw实际角 | 度 |
| 5 | Pitch目标角 | 度，仰为正 |
| 6 | Pitch实际角 | 度，仰为正 |
| 7 | 激光测距距离 | cm；-1表示无有效距离 |

比例使用官方yaw_ratio/pitch_ratio（1ms采样、alpha=0.001），并非累计平均分。退出轨迹后保留本轮比例，下一轮运行重新统计；刚上电无样本时为0。累计得分与故障诊断仍保留于RAM，不经串口发送。

帧：`AA BB 1C` + 7个little-endian float32 + 1字节校验，共32字节。
校验为28字节浮点数据累加取低8位，不含帧头和长度。
SerialPlot：Custom Frame、固定帧头`AA BB 1C`、Fixed Size=28、Float、7通道、Little Endian、Checksum开启。角度已在固件换为度，所有通道Gain=1、Offset=0。

载入`config/gimbal-readable-7ch.ini`。旧18通道固件和此配置不兼容，需要烧录配套新固件后验证收帧。旧18通道串口采集脚本不适用于新协议。
