#include "main.h"
#include "usart.h"
#include "TJC.h"
#include "stm32g4xx_it.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

void TJC_printf_num(const char id, int val)
{
    char buf[50];
    uint16_t len = snprintf(buf, sizeof(buf), "n%c.val=%d", id, val);
    HAL_UART_Transmit(&huart5, (uint8_t *)buf, len, 1000);

    // 单独发送结束符
    uint8_t end[3] = {0xff, 0xff, 0xff};
    HAL_UART_Transmit(&huart5, end, 3, 1000);
}

// 串口屏设置为三位小数，传入整数会自动把小数点左移三位
void TJC_printf_float(const char id, int val)
{
    char buf[50];
    uint16_t len = snprintf(buf, sizeof(buf), "x%c.val=%d", id, val);
    HAL_UART_Transmit(&huart5, (uint8_t *)buf, len, 1000);

    // 单独发送结束符
    uint8_t end[3] = {0xff, 0xff, 0xff};
    HAL_UART_Transmit(&huart5, end, 3, 1000);
}

void ScreenPrintf(const char *cmd, ...)
{
    uint8_t txt[30]; // 文本数组
    uint8_t *txt_p;  // 文本数组指针
    memset(&txt, 0, sizeof(txt));
    memset(&txt_p, 0, sizeof(txt_p));
    va_list args;        // 定义一个va_list类型的变量，用来储存单个参数
    va_start(args, cmd); // 使args指向可变参数的第一个参数
    vsprintf((char *)txt, (const char *)cmd, args);
    va_end(args);
    txt_p = txt;
    while (*txt_p != '\0')
    {
        HAL_UART_Transmit(&huart5, txt_p, 1, 1000); // 通过UART发送单个字节
        txt_p++;
    }
    uint8_t end[3] = {0xff, 0xff, 0xff};
    HAL_UART_Transmit(&huart5, end, 3, 1000);
}
// 使用时ScreenPrintf("n0.val=%d",MyData); //一次性发完命令和结束符
