#include "cmsis_os.h"
#include "usart.h"
#include "dma.h"
#include "tof_task.hpp"  // 对外接口声明(与 uart_task.cpp 共用, 保证签名一致)

// ==================== GY-56 红外测距任务 (规则书 4.4.2.1) ====================
// 接线: 第一阶段设计的 24V→5V 降压板 5V → GY-56 VCC, GND 三端共地;
//       GY-56 Pin3(CT / 串口 TX) → C 板 PG9(USART6_RX), Pin4(DR / 串口 RX) → PG14(USART6_TX);
//       PS 焊点保持出厂 ps=1(串口模式, 焊上 PS 才是 IIC)。
// CubeMX: USART6 = 9600 8N1, RX 走 DMA2_Stream2(DMA_NORMAL), 用空闲中断(IDLE)按帧间静默切帧。
//
// 帧(模块→MCU, 手册 §3.2, 8~13 字节):
//   [0]=0x5A [1]=0x5A [2]=0x15(帧类型) [3]=数据量 L
//   [4 .. 3+L] 数据: 前两字节 = 距离 cm 高字节/低字节, 第 3 字节 = 测量模式, 第 4 字节(仅 L=4) = 温度℃
//   [4+L] 校验和 = 前 (4+L) 字节累加取低 8 位
//   ⇒ 帧长 = 5 + L: L=3 → 8 字节(不带温度), L=4 → 9 字节(出厂默认)
//   手册例: 5A 5A 15 04 00 0A 02 1A F3 → 距离 10 cm, 模式 2(高精度), 温度 26℃, 校验 0xF3
//   ⚠️ 原实现把帧长写死 9 字节, L=3 的 8 字节帧会被误判校验错 —— 这里按 L 动态算长度。
//
// 三种"基本处理"(规则书 L364):
//   无效值(距离 0) / 超量程(> 1.2m) → tof_out_range++, 状态码 TOF_STATUS_RANGE
//   帧头、长度、校验错            → tof_bad_sum++,   状态码 TOF_STATUS_FRAME
//   通信异常(超时, 或串口错误)     → tof_timeouts/tof_errors++, 状态码 TOF_STATUS_STALE
//
// 注意: 串口回调 HAL_UARTEx_RxEventCallback 全工程只有一份(在 uart_task.cpp 里),
//       本文件只提供 tof_handle_rx()/tof_handle_error(), 由 uart_task.cpp 的 huart6 分支调用。

namespace
{
// 空闲中断一次可能收到不止一帧, 缓冲放大到 24 字节(单帧最长 13 字节, 留足余量),
// 解析器会在缓冲里逐帧消耗, 遇到噪声/残帧还能往后重新同步。
uint8_t tof_buf[24];

constexpr uint16_t TOF_BUF_SIZE = sizeof(tof_buf);
constexpr uint8_t TOF_HEAD0 = 0x5A;
constexpr uint8_t TOF_HEAD1 = 0x5A;
constexpr uint8_t TOF_FRAME_TYPE = 0x15;
constexpr uint8_t TOF_LEN_MIN = 1;                    // 数据量最少 1
constexpr uint8_t TOF_LEN_MAX = 8;                    // 数据量最多 8(手册: 帧长 8~13)
constexpr uint16_t TOF_FRAME_MIN = 5 + TOF_LEN_MIN;   // 最短帧 6 字节
constexpr uint16_t TOF_FRAME_MAX = 5 + TOF_LEN_MAX;   // 最长帧 13 字节

// 量程判据(超量程按无效处理): 出厂的高精度/快速/一般模式都是 1.2m = 120cm;
// 若用命令切到"长距离模式(mode 0, 量程 2m)", 把这里改成 200
constexpr uint16_t TOF_RANGE_CM = 120;

// 通信异常判据: 一般模式约 35ms 一帧, 500ms 没有新有效帧就判无效(规则书要求的"通信异常处理")
constexpr uint32_t TOF_TIMEOUT_MS = 500;

// 上电等待: 模块上电后要自己做完 SPADs 修正 + 温度修正才出数据, 等它稳定再发配置命令
constexpr uint32_t TOF_WARMUP_MS = 1000;

// 中值滤波窗口: 对量程内的有效原始值做 3 点中值, 压掉单帧跳变 = 规则书要求的"稳定读取"
constexpr uint8_t TOF_MED_N = 3;

// 配置命令(格式 0xA5 + 指令 + 指令值 + sum, sum = 前三字节累加取低 8 位), 都不保存(掉电恢复出厂):
//   一般模式 = 测量模式 3(约 35ms/帧, ±2cm) —— 出厂默认是高精度(约 200ms/帧), 太慢
//   连续输出 = 输出模式 1(默认值, 这里显式写一遍, 防止有人用 SET 引脚改过查询模式)
// 不保存的理由: 模块的"保存设置"会写 FLASH, 每次上电都写没有必要; 命令在 tof_init() 里重发即可。
const uint8_t TOF_CMD_MODE_GENERAL[] = {0xA5, 0x54, 0x03, 0xFC};  // 测量模式 = 3 一般
const uint8_t TOF_CMD_OUTPUT_CONT[] = {0xA5, 0x65, 0x01, 0x0B};   // 输出模式 = 1 连续
constexpr uint32_t TOF_CFG_RETRY_MS = 2000;  // 连续 2s 收不到有效帧 → 重发一次配置命令

uint32_t tof_cfg_ms = 0;        // 最近一次发配置命令的时刻
uint32_t tof_seen_frames = 0;   // 任务侧已经处理过的帧数(对应中断里的 tof_frames)
float tof_med[TOF_MED_N] = {0};
uint8_t tof_med_n = 0;  // 窗口里已填了几个
uint8_t tof_med_i = 0;  // 环形写指针
}  // namespace

// ===== 给其它任务看的全局量(折叠逻辑读 tof_distance_cm / tof_valid, 或调 tof_get_distance) =====
volatile float tof_distance_cm = -1.0f;
volatile float tof_raw_cm = -1.0f;
volatile bool tof_valid = false;
volatile uint8_t tof_status = TOF_STATUS_NO_FRAME;
volatile uint8_t tof_mode = 0xFF;
volatile int8_t tof_temp_c = -1;
volatile uint32_t tof_last_ms = 0;
volatile uint32_t tof_frames = 0;
volatile uint32_t tof_bad_sum = 0;
volatile uint32_t tof_out_range = 0;
volatile uint32_t tof_timeouts = 0;
volatile uint32_t tof_errors = 0;

// 3 点中值(窗口没填满就取现有的): 让折叠逻辑拿到稳定的距离, 而不是单帧跳变
// 距离数据的“年龄”(ms)：时间戳在串口中断里写，任务里比较。
// 必须用带符号差值：中断可能在任务读走 now_ms 之后才写时间戳，
// 无符号相减会算成 42 亿 → 偶发误判“超时”，把中值清掉、距离变 -1（折叠判据会打嗝）。
static uint32_t tof_age_ms(uint32_t now_ms)
{
  const int32_t dt = (int32_t)(now_ms - tof_last_ms);
  return (dt < 0) ? 0u : (uint32_t)dt;
}

static float tof_median()
{
  if (tof_med_n == 0) return -1.0f;

  float t[TOF_MED_N];
  for (uint8_t i = 0; i < tof_med_n; ++i) t[i] = tof_med[i];
  for (uint8_t i = 0; i < tof_med_n; ++i) {
    for (uint8_t j = (uint8_t)(i + 1); j < tof_med_n; ++j) {
      if (t[j] < t[i]) {
        const float s = t[i];
        t[i] = t[j];
        t[j] = s;
      }
    }
  }
  return t[tof_med_n / 2];
}

// 在 [b, b+n) 里找一帧(允许前面有噪声/残帧): 命中就填距离/模式/温度, 返回消耗掉的字节数;
// 没有完整有效帧时返回 0。返回"帧长 + 前面噪声"的长度, 于是调用方循环就能把一段数据里的多帧全吃掉。
static uint16_t tof_parse_one(const uint8_t* b, uint16_t n, float* cm, uint8_t* mode, int8_t* temp)
{
  for (uint16_t i = 0; (uint16_t)(i + TOF_FRAME_MIN) <= n; ++i) {
    if (b[i] != TOF_HEAD0 || b[i + 1] != TOF_HEAD1 || b[i + 2] != TOF_FRAME_TYPE) continue;

    const uint8_t len = b[i + 3];
    if (len < TOF_LEN_MIN || len > TOF_LEN_MAX) continue;  // 数据量非法 → 当噪声, 继续往后找帧头

    const uint16_t total = (uint16_t)(5 + len);  // 帧长 = 5 + 数据量
    if ((uint16_t)(i + total) > n) return 0;     // 这帧还没收全: 不消耗, 等下一批数据

    uint8_t sum = 0;
    for (uint16_t k = 0; k < (uint16_t)(4 + len); ++k) sum = (uint8_t)(sum + b[i + k]);
    if (sum != b[i + 4 + len]) continue;  // 校验和错 → 继续找下一个帧头

    *cm = (float)(((uint16_t)b[i + 4] << 8) | b[i + 5]);  // 距离高字节在前, 单位 cm
    *mode = b[i + 6];
    *temp = (len >= 4) ? (int8_t)b[i + 7] : -1;  // L=3 的 8 字节帧不带温度
    return (uint16_t)(i + total);                // 连前面的噪声一起消耗掉
  }
  return 0;
}

// uart_task.cpp 的空闲(或收满)中断回调里调用(huart6 分支) —— 中断上下文: 只解析、计数、重挂 DMA
void tof_handle_rx(uint16_t size)
{
  if (!TOF_ENABLE) return;  // 测距关闭: 不解析也不重挂 DMA

  if (size > TOF_BUF_SIZE) size = TOF_BUF_SIZE;  // 防御: HAL 不会给超过缓冲的长度

  uint16_t off = 0;
  while (off < size) {
    float cm = 0.0f;
    uint8_t mode = 0xFF;
    int8_t temp = -1;

    const uint16_t used = tof_parse_one(&tof_buf[off], (uint16_t)(size - off), &cm, &mode, &temp);
    if (used == 0) break;  // 这一段里再没有完整有效帧
    off = (uint16_t)(off + used);

    if (cm <= 0.0f || cm > (float)TOF_RANGE_CM) {
      // 无效值(距离 0)或超量程: 不当成有效距离, 计数 + 标记
      tof_out_range++;
      tof_status = TOF_STATUS_RANGE;
      continue;
    }

    tof_raw_cm = cm;
    tof_mode = mode;
    tof_temp_c = temp;
    tof_last_ms = osKernelSysTick();
    tof_valid = true;
    tof_frames++;
    tof_status = TOF_STATUS_OK;
  }

  if (off == 0 && size > 0) {
    // 一段数据里连一个完整有效帧都没有: 帧头/长度/校验错
    tof_bad_sum++;
    if (tof_status != TOF_STATUS_RANGE) tof_status = TOF_STATUS_FRAME;
  }

  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, tof_buf, TOF_BUF_SIZE);  // 重新挂 DMA
}

// uart_task.cpp 的错误回调里调用(huart6 分支): 帧错误/溢出等, 重新挂 DMA
void tof_handle_error()
{
  if (!TOF_ENABLE) return;
  tof_errors++;
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, tof_buf, TOF_BUF_SIZE);
}

// 下发配置命令(阻塞发送, 4 字节 @9600 约 4.2ms; 只在开机和长时间收不到数据时调用)
static void tof_send_cfg()
{
  HAL_UART_Transmit(&huart6, (uint8_t*)TOF_CMD_MODE_GENERAL, (uint16_t)sizeof(TOF_CMD_MODE_GENERAL), 20);
  HAL_UART_Transmit(&huart6, (uint8_t*)TOF_CMD_OUTPUT_CONT, (uint16_t)sizeof(TOF_CMD_OUTPUT_CONT), 20);
  tof_cfg_ms = osKernelSysTick();
}

static void tof_init()
{
  if (!TOF_ENABLE) return;
  tof_send_cfg();  // 先切"一般模式 + 连续输出", 再开始收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, tof_buf, TOF_BUF_SIZE);
}

static void tof_update(uint32_t now_ms)
{
  if (!TOF_ENABLE) return;

  // 1) 有新帧 → 更新 3 点中值(规则书 L364 "稳定读取")
  if (tof_frames != tof_seen_frames) {
    tof_seen_frames = tof_frames;
    tof_med[tof_med_i] = tof_raw_cm;
    tof_med_i = (uint8_t)((tof_med_i + 1) % TOF_MED_N);
    if (tof_med_n < TOF_MED_N) tof_med_n++;
    tof_distance_cm = tof_median();
  }

  // 2) 通信异常: 超过 TOF_TIMEOUT_MS 没有新有效帧 → 判无效 + 计数(规则书 L364 的"通信异常处理")
  if (tof_valid && tof_age_ms(now_ms) > TOF_TIMEOUT_MS) {
    tof_valid = false;
    tof_distance_cm = -1.0f;
    tof_raw_cm = -1.0f;
    tof_status = TOF_STATUS_STALE;
    tof_timeouts++;
    tof_med_n = 0;  // 重新攒中值, 不拿超时之前的旧数据
    tof_med_i = 0;
  }

  // 3) 一直收不到有效帧 → 重发配置命令(模块上电比我们晚 / 被别的命令改过模式 / 线没接好)
  if (!tof_valid && (uint32_t)(now_ms - tof_cfg_ms) > TOF_CFG_RETRY_MS) {
    // 顺便补一次 DMA 重挂：万一半满/错误回调里那次重挂失败(返回 HAL_BUSY)，测距会永久停住。
    // 已经在收的时候这调用返回 HAL_BUSY，不会打断正常接收。
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, tof_buf, TOF_BUF_SIZE);
    tof_send_cfg();
  }
}

// 给云台自动折叠逻辑(4.4.7)的统一入口
bool tof_get_distance(float* cm, uint32_t now_ms)
{
  if (!TOF_ENABLE || !tof_valid) return false;
  if (tof_distance_cm < 0.0f) return false;
  if (tof_age_ms(now_ms) > TOF_TIMEOUT_MS) return false;
  if (cm != nullptr) *cm = tof_distance_cm;
  return true;
}

extern "C" void tof_task()
{
  if (!TOF_ENABLE) {
    // 测距关闭: 任务保留(接线后把 TOF_ENABLE 改成 true 即可用), 不碰串口、不占 CPU
    for (;;) osDelay(1000);
  }

  osDelay(TOF_WARMUP_MS);  // 等模块上电自检(SPADs 修正/温度修正)完成
  tof_init();

  while (true) {
    tof_update(osKernelSysTick());
    osDelay(10);
  }
}
