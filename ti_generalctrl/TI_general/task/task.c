#include "task.h"
#include "PID.h"
#include "drv_math.h"
#include "ti_msp_dl_config.h"

FollowControl follow_control; // 小车运动状态控制器
// 初始化follow_control
FollowControl follow_control = {{0, 0}, 0, 0, 0};
/*
 * @brief  任务初始化函数
 *         初始化PID控制器、传感器、按键等
 * @param  None
 * @retval None
 * @note
 * 该函数在系统启动时调用，确保所有必要的组件都已正确在cubemx配置和初始化。并且如果卡死属于是初始化没有找到该设备，hal库死循环了。
 */
void task_init(void) {

  // 调节pid，注意所有函数P和D正负号对应的输出结果就是正负的,p和d是同号的

  // 角度度环和速度环的PID控制器初始化
  Angle.pid_init(&Angle, 130, 150.0f / 200.0f, 0.5, 0, 0, 1000, 10000, 1, 0, 0,
                 0, 0, 0, PID_D_First_DISABLE, PID_I_Separate_DISABLE,
                 PID_I_Variable_Speed_DISABLE); // 转向环

  // p[0,100],d[0,1000]
  Follow.pid_init(&Follow, 11, 0, 0, 0, 0, 1.5, 5000, 1, 0, 0, 0, 0, 0,
                  PID_D_First_DISABLE, PID_I_Separate_ENABLE,
                  PID_I_Variable_Speed_DISABLE); // 位置环


  NVIC_ClearPendingIRQ(Pid_INST_INT_IRQN);
  NVIC_EnableIRQ(Pid_INST_INT_IRQN);

  NVIC_ClearPendingIRQ(TJC_INST_INT_IRQN);
  NVIC_EnableIRQ(TJC_INST_INT_IRQN);

  // DL_TimerA_setCaptureCompareValue(Motor_INST, 5000, GPIO_Motor_C0_IDX);
  // DL_TimerA_setCaptureCompareValue(Motor_INST, 5000, GPIO_Motor_C1_IDX);

  DL_TimerA_startCounter(Motor_INST);
}
