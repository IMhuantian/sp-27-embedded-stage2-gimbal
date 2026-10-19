#include "gimbal_task.hpp"

#include "cmsis_os.h"
#include "can_diag.hpp"
#include "gm6020_fold_test.hpp"
#include "tools/mahony/mahony.hpp"

// 定义在 imu_task.cpp
extern sp::Mahony imu;

// SerialPlot 14 通道；测试/诊断模式的具体含义见 README。
float plot_ch[14] = {0.0f};

// 双 CAN 接收：按实际 hcan 分发到 can_diag；测试模式通过 frame hook 取 0x205。
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  can_diag::can_diag_on_rx(hcan);
}

extern "C" void gimbal_task()
{
#if FOLD_TEST_ENABLE
  gm6020_fold_test::gm6020_fold_test_loop(plot_ch);
  return;
#endif

#if CAN_DIAG_ENABLE
  can_diag::can_diag_loop(plot_ch);
  return;
#endif

  // 两者都关闭时没有云台控制源，保持空转。
  while (true) {
    osDelay(1);
  }
}

float gimbal_yaw_rad()
{
  return imu.yaw;
}

float gimbal_pitch_rad()
{
  return imu.pitch;
}
