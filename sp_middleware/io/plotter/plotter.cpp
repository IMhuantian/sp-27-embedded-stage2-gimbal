#include "plotter.hpp"

namespace sp
{
Plotter::Plotter(UART_HandleTypeDef * huart, bool use_dma)
: huart_(huart), use_dma_(use_dma), hal_status_(HAL_OK)
{
}

// 加法校验：SerialPlot 收到后逐字节求和比对，不一致就丢帧并重新同步。
// 覆盖 size 字节 + 有效数据字节（不含帧头 AA BB 与校验字节本身）。
void Plotter::update_checksum()
{
  const uint8_t * first = PLOTTER_CHECKSUM_INCLUDE_SIZE
                            ? reinterpret_cast<const uint8_t *>(&plot_frame_.size)
                            : reinterpret_cast<const uint8_t *>(plot_frame_.data);
  const uint8_t * last =
    reinterpret_cast<const uint8_t *>(plot_frame_.data) + plot_frame_.size;
  uint8_t sum = 0;
  for (const uint8_t * p = first; p < last; ++p) sum = static_cast<uint8_t>(sum + *p);
  plot_frame_.checksum = sum;
}

void Plotter::plot(float value1)
{
  static_assert(PLOTTER_FLOAT_NUM >= 1);
  plot_frame_.size = 4 * 1;
  plot_frame_.data[0] = value1;
  send();
}

void Plotter::plot(float value1, float value2)
{
  static_assert(PLOTTER_FLOAT_NUM >= 2);
  plot_frame_.size = 4 * 2;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  send();
}

void Plotter::plot(float value1, float value2, float value3)
{
  static_assert(PLOTTER_FLOAT_NUM >= 3);
  plot_frame_.size = 4 * 3;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  send();
}

void Plotter::plot(float value1, float value2, float value3, float value4)
{
  static_assert(PLOTTER_FLOAT_NUM >= 4);
  plot_frame_.size = 4 * 4;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  send();
}

void Plotter::plot(float value1, float value2, float value3, float value4, float value5)
{
  static_assert(PLOTTER_FLOAT_NUM >= 5);
  plot_frame_.size = 4 * 5;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6)
{
  static_assert(PLOTTER_FLOAT_NUM >= 6);
  plot_frame_.size = 4 * 6;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7)
{
  static_assert(PLOTTER_FLOAT_NUM >= 7);
  plot_frame_.size = 4 * 7;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8)
{
  static_assert(PLOTTER_FLOAT_NUM >= 8);
  plot_frame_.size = 4 * 8;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9)
{
  static_assert(PLOTTER_FLOAT_NUM >= 9);
  plot_frame_.size = 4 * 9;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10)
{
  static_assert(PLOTTER_FLOAT_NUM >= 10);
  plot_frame_.size = 4 * 10;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11)
{
  static_assert(PLOTTER_FLOAT_NUM >= 11);
  plot_frame_.size = 4 * 11;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11, float value12)
{
  static_assert(PLOTTER_FLOAT_NUM >= 12);
  plot_frame_.size = 4 * 12;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  plot_frame_.data[11] = value12;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11, float value12, float value13)
{
  static_assert(PLOTTER_FLOAT_NUM >= 13);
  plot_frame_.size = 4 * 13;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  plot_frame_.data[11] = value12;
  plot_frame_.data[12] = value13;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11, float value12, float value13,
  float value14)
{
  static_assert(PLOTTER_FLOAT_NUM >= 14);
  plot_frame_.size = 4 * 14;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  plot_frame_.data[11] = value12;
  plot_frame_.data[12] = value13;
  plot_frame_.data[13] = value14;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11, float value12, float value13,
  float value14, float value15)
{
  static_assert(PLOTTER_FLOAT_NUM >= 15);
  plot_frame_.size = 4 * 15;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  plot_frame_.data[11] = value12;
  plot_frame_.data[12] = value13;
  plot_frame_.data[13] = value14;
  plot_frame_.data[14] = value15;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11, float value12, float value13,
  float value14, float value15, float value16)
{
  static_assert(PLOTTER_FLOAT_NUM >= 16);
  plot_frame_.size = 4 * 16;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  plot_frame_.data[11] = value12;
  plot_frame_.data[12] = value13;
  plot_frame_.data[13] = value14;
  plot_frame_.data[14] = value15;
  plot_frame_.data[15] = value16;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11, float value12, float value13,
  float value14, float value15, float value16, float value17)
{
  static_assert(PLOTTER_FLOAT_NUM >= 17);
  plot_frame_.size = 4 * 17;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  plot_frame_.data[11] = value12;
  plot_frame_.data[12] = value13;
  plot_frame_.data[13] = value14;
  plot_frame_.data[14] = value15;
  plot_frame_.data[15] = value16;
  plot_frame_.data[16] = value17;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11, float value12, float value13,
  float value14, float value15, float value16, float value17, float value18)
{
  static_assert(PLOTTER_FLOAT_NUM >= 18);
  plot_frame_.size = 4 * 18;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  plot_frame_.data[11] = value12;
  plot_frame_.data[12] = value13;
  plot_frame_.data[13] = value14;
  plot_frame_.data[14] = value15;
  plot_frame_.data[15] = value16;
  plot_frame_.data[16] = value17;
  plot_frame_.data[17] = value18;
  send();
}

void Plotter::send()
{
  update_checksum();
  const uint16_t length = sizeof(plot_frame_.start) + sizeof(plot_frame_.size) +
                          plot_frame_.size + sizeof(plot_frame_.checksum);
  if (use_dma_) {
    hal_status_ = HAL_UART_Transmit_DMA(huart_, (uint8_t *)&plot_frame_, length);
  }
  else {
    hal_status_ = HAL_UART_Transmit(huart_, (uint8_t *)&plot_frame_, length, 0xff);
  }
}

}  // namespace sp
