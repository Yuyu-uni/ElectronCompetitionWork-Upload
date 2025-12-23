#ifndef TJC_H
#define TJC_H

#include <stdint.h>
#include "usart.h" // 确保包含USART头文件

// 枚举串口屏返回的所有数据对应的状态，包括写入和读取
typedef enum
{
    TJC_OK = 0, // 成功
    TJC_Write,  // 写入数据
    TJC_Read,   // 读取数据
    TJC_Error   // 错误
} TJC_Status;

void TJC_printf_num(const char id, int val);
void TJC_printf_float(const char id, int val);
void ScreenPrintf(const char *cmd, ...);

#endif // !TJC_H
