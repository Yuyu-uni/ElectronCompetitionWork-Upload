/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "Motor.h"
#include "PID.h"
#include "task.h"
#include "ti_msp_dl_config.h"

int main(void) {
  SYSCFG_DL_init();
  task_init();

  // while(follow_control.state == 0)
  NVIC_ClearPendingIRQ(Cac_INST_INT_IRQN);
  NVIC_EnableIRQ(Cac_INST_INT_IRQN); // 开始圈数记录
  DL_TimerA_startCounter(Motor_INST);
  follow_control.location = 1; // 开始圈数记录
  follow_control.base_speed = 1000;
  // 在主循环里一直读取循迹模块的值并分析
  volatile uint8_t follow_sensor_data; // 8位变量存储循迹传感器数据

  while (1) {
    // 控制走多少圈
    if (follow_control.rote[1] >= follow_control.rote[0]) {
      Load(0, 0);
    }
    //巡线模式
    Follow_direct();
    //进入直角转弯
    if (follow_control.state == 2) {
      DL_TimerA_stopCounter(Pid_INST);
      Load(1500,-1500);
      while(Follow_num >=3)
      {
        Follow_direct();
      }
      //回到巡线模式
        DL_TimerA_startCounter(Motor_INST);
      follow_control.state = 1;
    }
  }
}
