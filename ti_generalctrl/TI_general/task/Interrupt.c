#include "Motor.h"
#include "PID.h"
#include "stdio.h"
#include "string.h"
#include "task.h"
#include "ti_msp_dl_config.h"

void Pid_INST_IRQHandler(void) {
  volatile float Followpid_output =
  pid_calculate(&Follow, 0, 0); // 位置环 PID计算
  Load((int)(follow_control.base_speed + Followpid_output),
       (int)(follow_control.base_speed - Followpid_output));
}

void Cac_INST_IRQHandler(void){
  follow_control.location = 1;//重新开始记录圈数
}

// 串口发送字符串
// 串口发送单个字符
void uart0_send_char(char ch) {
  // 当串口0忙的时候等待，不忙的时候再发送传进来的字符
  while (DL_UART_isBusy(TJC_INST) == true)
    ;
  // 发送单个字符
  DL_UART_Main_transmitData(TJC_INST, ch);
}

void uart0_send_string(char *str) {
  // 当前字符串地址不在结尾 并且 字符串首地址不为空
  while (*str != 0 && str != 0) {
    // 发送字符串首地址中的字符，并且在发送完成之后首地址自增
    uart0_send_char(*str++);
  }
}
void ScreenPrintf(const char *cmd, ...) {
  uint8_t txt[33]; // 文本数组，预留3字节结尾
  uint8_t *txt_p;  // 文本数组指针
  memset(txt, 0, sizeof(txt));
  va_list args;
  va_start(args, cmd);
  int len = vsprintf((char *)txt, cmd, args);
  va_end(args);

  // 拼接三个0xff到文本末尾
  txt[len++] = 0xff;
  txt[len++] = 0xff;
  txt[len++] = 0xff;

  txt_p = txt;
  for (int i = 0; i < len; i++) {
    uart0_send_char(txt_p[i]);
  }
}
// 使用时ScreenPrintf("n0.val=%d",MyData); //一次性发完命令和结束符

void TJC_INST_IRQHandler(void) {
  // 接收到字符1~5，表示转1~5圈。收到‘S’表示启动，收到‘E’直接将E发出去
  int uart_data = DL_UART_Main_receiveData(TJC_INST);
  if (uart_data >= '1' && uart_data <= '5') {
    follow_control.rote[0] = uart_data - '0'; // 记录路线
    // 打印当前路线到TJC
    ScreenPrintf("n0.val=%d", uart_data - '0');
  } else if (uart_data == 'S') {
    follow_control.state = 1; // 启动状态
  } else if (uart_data == 'E') {
    DL_UART_Main_transmitData(TJC_INST, uart_data); // 直接将E发出去
  } else if (uart_data == '0') {
    follow_control.rote[0] = 0; // 清零路线
    ScreenPrintf("n0.val=%d", uart_data - '0');
  }
}

void Following_INST_IRQHandler(void)
{
  switch(DL_TimerA_getPendingInterrupt(Following_INST))
  {
    case DL_TIMER_IIDX_ZERO:
      break;

    default:
      break;
  }
}