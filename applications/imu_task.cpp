#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"

// bmi088 坐标系到机体系的旋转矩阵。
// 按"C 板横装、CAN 一侧朝前"填，装法变了要跟着改，怎么定见 sp_middleware/io/bmi088/readme.md
const float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

// BMI088 走 SPI1: SCK=PB3 / MISO=PB4 / MOSI=PA7，片选 加速度计 PA4、陀螺仪 PB0
sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);

// dt 要跟这个任务的实际周期对上：osDelay(1) + 1000Hz tick，也就是 1ms
sp::Mahony imu(1e-3f);

extern "C" void imu_task()
{
  bmi088.init();

  while (true) {
    bmi088.update();
    imu.update(bmi088.acc, bmi088.gyro);
    osDelay(1);
  }
}
