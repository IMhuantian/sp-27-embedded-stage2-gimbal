#ifndef SP__PLOTTER_HPP
#define SP__PLOTTER_HPP

#include "usart.h"

namespace sp
{
// 帧格式：AA BB <size=72> <18×float> <校验1字节>，共 76 字节。
// SerialPlot 的 CustomFrame 勾选 checksum 后，最后 1 字节按累加和校验，坏帧会被丢弃并重新同步，
// 不会再画出 8e35 这种错位解出来的尖峰。
constexpr size_t PLOTTER_FLOAT_NUM = 18;
// 校验覆盖范围：true = 含 size 字节（从 size 到数据末尾）；false = 只算 72 字节数据。
constexpr bool PLOTTER_CHECKSUM_INCLUDE_SIZE = true;

struct __attribute__((packed)) PlotFrame
{
  uint8_t start[2] = {0xAA, 0xBB};
  uint8_t size;
  float data[PLOTTER_FLOAT_NUM];
  uint8_t checksum = 0;
};

class Plotter
{
public:
  Plotter(UART_HandleTypeDef * huart, bool use_dma = true);

  void plot(float value1);
  void plot(float value1, float value2);
  void plot(float value1, float value2, float value3);
  void plot(float value1, float value2, float value3, float value4);
  void plot(float value1, float value2, float value3, float value4, float value5);
  void plot(float value1, float value2, float value3, float value4, float value5, float value6);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12,
    float value13);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12,
    float value13, float value14);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12,
    float value13, float value14, float value15);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12,
    float value13, float value14, float value15, float value16);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12,
    float value13, float value14, float value15, float value16, float value17);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12,
    float value13, float value14, float value15, float value16, float value17, float value18);

private:
  UART_HandleTypeDef * huart_;
  bool use_dma_;
  HAL_StatusTypeDef hal_status_;
  PlotFrame plot_frame_;

  void send();
  void update_checksum();
};

}  // namespace sp

#endif  // SP__PLOTTER_HPP