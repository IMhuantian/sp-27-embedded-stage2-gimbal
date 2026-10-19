#include "can_diag.hpp"

#include "cmsis_os.h"

namespace can_diag
{
volatile uint32_t state = 0;
BusStats buses[kBusCount];
volatile uint32_t last_unknown_id = 0;
volatile uint32_t last_unknown_bus = 0;

namespace
{
constexpr uint32_t kCan1FilterBank = 0;
constexpr uint32_t kCan2FilterBank = 14;  // CAN2 滤波器从 bank 14 开始

FrameHook frame_hook = nullptr;

int find_candidate(uint32_t id)
{
  for (int i = 0; i < kCandidateCount; ++i) {
    if (kCandidateIds[i] == id) return i;
  }
  return -1;
}

void record_unknown(BusStats & bus, uint32_t id, uint32_t now_ms)
{
  for (int i = 0; i < kUnknownSlots; ++i) {
    if (bus.unknown[i].used != 0 && bus.unknown[i].id == id) {
      bus.unknown[i].count++;
      bus.unknown[i].last_ms = now_ms;
      return;
    }
  }
  for (int i = 0; i < kUnknownSlots; ++i) {
    if (bus.unknown[i].used == 0) {
      bus.unknown[i].used = 1;
      bus.unknown[i].id = id;
      bus.unknown[i].count = 1;
      bus.unknown[i].last_ms = now_ms;
      return;
    }
  }
  bus.unknown_overflow++;
}

bool configure_bus(CAN_HandleTypeDef * hcan, uint32_t filter_bank, BusStats & bus)
{
  bus.init_ok = false;
  bus.start_ok = false;

  CAN_FilterTypeDef filter = {};
  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterBank = filter_bank;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK) return false;
  bus.init_ok = true;

  // 只启动接收；不发送任何帧。
  if (HAL_CAN_Start(hcan) != HAL_OK) return false;
  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) return false;
  bus.start_ok = true;
  return true;
}

void can_diag_init()
{
  for (int b = 0; b < kBusCount; ++b) {
    for (int c = 0; c < kCandidateCount; ++c) {
      buses[b].candidates[c].id = kCandidateIds[c];
    }
  }

  // CAN1 / CAN2 都用 1 Mbps（CubeMX 的 Prescaler/BS1/BS2 相同）。
  // CAN2 的 MspInit 会同时使能 CAN1 时钟（见 Src/can.c），所以先初始化 CAN1 再初始化 CAN2。
  const bool can1_ok = configure_bus(&hcan1, kCan1FilterBank, buses[0]);
  const bool can2_ok = configure_bus(&hcan2, kCan2FilterBank, buses[1]);
  state = (can1_ok && can2_ok) ? 1u : 2u;
}

uint32_t online_mask_for(int bus, uint32_t now_ms)
{
  if (bus < 0 || bus >= kBusCount) return 0;
  uint32_t mask = 0;
  for (int c = 0; c < kCandidateCount; ++c) {
    const CandidateStats & cs = buses[bus].candidates[c];
    if (cs.last_valid_ms == 0) continue;
    if ((now_ms - cs.last_valid_ms) < kOnlineTimeoutMs) mask |= (1u << c);
  }
  return mask;
}

void fill_plot(float * plot, uint32_t now_ms)
{
  if (plot == nullptr) return;

  uint32_t online[kBusCount] = {0, 0};
  uint32_t oldest_age[kBusCount] = {0, 0};

  for (int b = 0; b < kBusCount; ++b) {
    online[b] = online_mask_for(b, now_ms);
    for (int c = 0; c < kCandidateCount; ++c) {
      const CandidateStats & cs = buses[b].candidates[c];
      if (cs.last_valid_ms == 0) continue;
      const uint32_t age = now_ms - cs.last_valid_ms;
      if (age < kOnlineTimeoutMs && age > oldest_age[b]) oldest_age[b] = age;
    }
  }

  plot[0] = (float)buses[0].total_frames;
  plot[1] = (float)buses[1].total_frames;
  plot[2] = (float)buses[0].last_id;
  plot[3] = (float)buses[1].last_id;
  plot[4] = (float)online[0];
  plot[5] = (float)online[1];
  plot[6] = (float)buses[0].valid_frames;
  plot[7] = (float)buses[1].valid_frames;
  plot[8] = (float)oldest_age[0];
  plot[9] = (float)oldest_age[1];
  plot[10] = (float)(buses[0].unknown_frames + buses[0].invalid_frames);
  plot[11] = (float)(buses[1].unknown_frames + buses[1].invalid_frames);
  plot[12] = (float)state;
  plot[13] = (float)last_unknown_id;
}
}  // namespace

void set_frame_hook(FrameHook hook)
{
  frame_hook = hook;
}

bool can_diag_start()
{
  can_diag_init();
  return state == 1u;
}

uint32_t online_mask(int bus, uint32_t now_ms)
{
  return online_mask_for(bus, now_ms);
}

void can_diag_on_rx(CAN_HandleTypeDef * hcan)
{
  int bus_index = -1;
  if (hcan == &hcan1)
    bus_index = 0;
  else if (hcan == &hcan2)
    bus_index = 1;
  else
    return;

  BusStats & bus = buses[bus_index];
  const uint32_t now_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    CAN_RxHeaderTypeDef header = {};
    uint8_t data[8] = {0};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
      bus.rx_errors++;
      state = 3u;
      break;
    }

    bus.total_frames++;
    bus.last_frame_ms = now_ms;

    const bool standard = (header.IDE == CAN_ID_STD);
    const uint32_t id = standard ? header.StdId : header.ExtId;
    bus.last_id = id;

    if (frame_hook != nullptr) {
      Frame frame = {
        bus_index,
        id,
        standard,
        header.RTR == CAN_RTR_REMOTE,
        (uint8_t)header.DLC,
        data,
        now_ms,
      };
      frame_hook(frame);
    }

    if (standard) {
      const int candidate = find_candidate(id);
      if (candidate >= 0) {
        CandidateStats & cs = bus.candidates[candidate];
        const bool valid = (header.RTR == CAN_RTR_DATA && header.DLC == 8);
        if (valid) {
          cs.valid_count++;
          cs.last_valid_ms = now_ms;
          bus.valid_frames++;
        }
        else {
          cs.invalid_count++;
          bus.invalid_frames++;
        }
      }
      else {
        bus.unknown_frames++;
        bus.last_unknown_id = id;
        last_unknown_id = id;
        last_unknown_bus = (uint32_t)(bus_index + 1);
        record_unknown(bus, id, now_ms);
      }
    }
    else {
      // 扩展帧不是标准反馈；只计数，不当作候选电机。
      bus.unknown_frames++;
      bus.last_unknown_id = id;
      last_unknown_id = id;
      last_unknown_bus = (uint32_t)(bus_index + 1);
    }
  }
}

void can_diag_loop(float * plot)
{
  can_diag_start();

  while (true) {
    fill_plot(plot, osKernelSysTick());
    osDelay(1);
  }
}
}  // namespace can_diag
