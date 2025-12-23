#include "ZX_servo.h"
#include "z_kinematics.h"
#include <string.h> 

// 定义总线舵机使用的UART句柄
#define ZX_SERVO_SIG_UART_HANDLE huart3

extern TIM_HandleTypeDef htim15; // Assuming TIM15 is used for servo control
extern UART_HandleTypeDef huart3; // 总线舵机使用的UART句柄

/*
 * @brief  该函数将串口3半双工模式的发送打包为一个函数，在函数中进行以下操作：
 *         1. 中止任何正在进行的传输
 *         2. 启用半双工发送模式
 *         3. 发送数据
 * @param  pData: 指向要发送的数据缓冲区的指针
 * @param  Size: 要发送的数据长度
 * @param  Timeout: 超时时间
 * @note   该函数为私有函数，不在.h文件中声明。
 */
void UART_HalfDuplex_Transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    HAL_UART_Abort(&ZX_SERVO_SIG_UART_HANDLE);                          // 先中止任何正在进行的传输
    HAL_HalfDuplex_EnableTransmitter(&ZX_SERVO_SIG_UART_HANDLE);        // 启用半双工发送模式
    HAL_UART_Transmit(&ZX_SERVO_SIG_UART_HANDLE, pData, Size, Timeout); // 发送数据
}
/*
 * @brief  该函数将串口3半双工模式的接收打包为一个函数，在函数中进行以下操作：
 *         1. 中止任何正在进行的传输
 *         2. 启用半双工接收模式
 *         3. 接收数据
 * @param  pData: 指向接收数据缓冲区的指针
 * @param  Size: 要接收的数据长度
 * @param  Timeout: 超时时间
 * @note   该函数为私有函数，不在.h文件中声明。
 */
void UART_HalfDuplex_Receive(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    HAL_UART_Abort(&ZX_SERVO_SIG_UART_HANDLE);                         // 先中止任何正在进行的传输
    HAL_HalfDuplex_EnableReceiver(&ZX_SERVO_SIG_UART_HANDLE);          // 启用半双工接收模式
    HAL_UART_Receive(&ZX_SERVO_SIG_UART_HANDLE, pData, Size, Timeout); // 接收数据
}

/*
 * @brief  初始化ZX_SERVO设备
 * @param  servo: 指向ZX_SERVO_Struct结构体的指针
 * @param  id: 设备ID
 * @param  mode: 工作模式**注意**是char类型'1' '2' '3' '4'
 * @note   此函数将设置设备ID，发送模式设置命令，并根据响应设置角度范围。
 */
void ZX_SERVO_Init(ZX_SERVO_Struct *servo, uint8_t id, char mode)
{
    servo->id = id;     // 设置设备ID
    servo->mode = mode; // 设置工作模式
    char mode_set_command[15];
    sprintf(mode_set_command, "#%03dPMOD%c!\r\n", servo->id, mode);                       // 格式化模式设置命令，该函数命名完后会自动添加\0
    UART_HalfDuplex_Transmit((uint8_t *)mode_set_command, sizeof(mode_set_command), 100); // 发送模式设置命令
    // HAL_UART_Transmit(&ZX_SERVO_SIG_UART_HANDLE, (uint8_t *)mode_set_command, sizeof(mode_set_command), 100); // 发送模式设置命令

    switch (servo->mode) // 根据模式进行不同的初始化
    {
    case '1':
        servo->angle_range = -270; // 设置角度范围为-270度--以逆时针为正方向
        break;
    case '2':
        servo->angle_range = 270; // 设置角度范围为270度
        break;
    case '3':
        servo->angle_range = -180; // 设置角度范围为-180度
        break;
    case '4':
        servo->angle_range = 180; // 设置角度范围为180度
        break;
    default:
        break;
    }
}
/*
 * @brief  设置ZX_SERVO设备的目标角度
 * @param  servo: 指向ZX_SERVO_Struct结构体的指针
 * @param  target_angle: 目标角度--注意以逆时针为正，顺时针为负
 * @note   此函数将目标角度转换为PWM波，并发送设置命令。(内部已进行限幅处理，确保PWM值在500到2500之间)
 */
void ZX_SERVO_SetAngle(ZX_SERVO_Struct *servo, int16_t target_angle)
{
    //-----角度计算->PWM波
    // 将角度范围映射到PWM值范围(500-2500)
    // 公式：PWM = 1500 + (target_angle * 1000) / abs(angle_range)
    uint16_t pwm_value;
    int16_t abs_range = (servo->angle_range > 0) ? servo->angle_range : -(servo->angle_range);
    
    // 限制目标角度在有效范围内
    if (servo->angle_range > 0) {
        // 正向范围 (0 到 +angle_range)
        if (target_angle < 0) target_angle = 0;
        if (target_angle > servo->angle_range) target_angle = servo->angle_range;
    } else {
        // 负向范围 (-abs_range 到 0) 或双向范围
        if (target_angle < servo->angle_range) target_angle = servo->angle_range;
        if (target_angle > abs_range) target_angle = abs_range;
    }
    
    // 计算PWM值：中位1500us，±1000us的范围
    pwm_value = 1500 + (target_angle * 1000) / abs_range;
    
    // 确保PWM值在有效范围内
    if (pwm_value < 500) pwm_value = 500;
    if (pwm_value > 2500) pwm_value = 2500;
    
    char angle_set_command[20];
    sprintf(angle_set_command, "#%03dP%04dT0100!\r\n", servo->id, pwm_value); // 格式化角度设置命令---默认T100-》即100ms完成旋转

    UART_HalfDuplex_Transmit((uint8_t *)angle_set_command, strlen(angle_set_command), 100); // 发送角度设置命令
}

void Servo_Init(void)
{
    // Initialize the servo motor
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); // Start PWM on TIM1 Channel 1
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2); // Start PWM on TIM1 Channel 2
}
// id1为底下那个270°控制x轴，id2为上面那个180°控制y轴
void Servo_SetAngle(uint8_t servo_id, uint16_t angle)
{
    uint16_t PWM_value;
    if (servo_id == 1)
    {
        PWM_value = 50 + (angle / 270.0) * 200;
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, PWM_value);
    }
    else if (servo_id == 2)
    {
        PWM_value = 50 + (angle / 180.0) * 200;
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, PWM_value);
    }
}
