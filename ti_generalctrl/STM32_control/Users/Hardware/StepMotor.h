#ifndef __STEPMOTOR_H__
#define __STEPMOTOR_H__

/*
 * 该宏定义定义步进电机的驱动方式
 * _STEP_DIR_CTRL_MODE_  - step(PWM)&dir(GPIO)控制方式
 * _CAN_CTRL_MODE_       - CAN总线控制方式
 */
#define _CAN_CTRL_MODE_

/**
 * 该宏定义确定步进电机驱动器的固件版本 分为Emm和X
 * _CAN_CTRL_FRAMEVER_X - X固件版本（目前支持更好）
 * _CAN_CTRL_FRAMEVER_Emm - Emm固件版本
 */
#define _CAN_CTRL_FRAMEVER_X

#define STEP_MOTOR_PULSE_ANGLE 1.8f // 每次脉冲转动的角度 单位为度

#include "stm32g474xx.h"
#include "stdio.h"

#ifdef _STEP_DIR_CTRL_MODE_
#include "tim.h"
#include "gpio.h"
enum StepMotorTag
{
    StepMotorA = 0,
    StepMotorB = 1
};

typedef struct
{
    uint16_t dividerMode; // 细分模式 每次脉冲->转动 STEP_MOTOR_PULSE_ANGLE/DividerMode 度

    TIM_HandleTypeDef stepTimHandler; // PWM Timer
    uint32_t stepTimChannel;          // PWM通道
    uint16_t dirPin;                  // 方向引脚
    GPIO_TypeDef *dirPinPort;         // 方向引脚端口

    double currtAngle;       // 初始角度
    double anglePerStep;     // 每次脉冲转动的角度 单位为度
    int32_t stepCounter;     // 步进计数器
    uint8_t stepTurningFlag; // 步进转动标志位 0-未转动 1-转动中 2-处于定速转动模式
} StepMotorStruct;

void StepMotor_Init(StepMotorStruct *motor, enum StepMotorTag motorTag, uint16_t dividerMode);

void StepMotor_SetSpeed(StepMotorStruct *motor, uint16_t speed);

void StepMotor_Turn(StepMotorStruct *motor, double angle);

void StepMotor_Tick(enum StepMotorTag motorTag);

void StepMotor_StartTurning(StepMotorStruct *motor, int16_t motorSpeed);

void StepMotor_Stop(StepMotorStruct *motor);

void StepMotor_SetAngle(StepMotorStruct *motor, double angle);
#endif // _STEP_DIR_CTRL_MODE_

#ifdef _CAN_CTRL_MODE_
#include "fdcan.h"

#define StepMotorNum 1 // 步进电机数量

#define ALL_STEPMOTOR_CTRL 0x00000000 // 同时控制所有步进电机的控制ID

#define STEPMOTOR_DIRECT_FORWARD 0x00  // 电机正转 右手定则 方向沿着电机轴方向向外
#define STEPMOTOR_DIRECT_BACKWARD 0x01 // 电机反转 右手定则 方向沿着电机轴方向向内

/**
 * @defgroup StepMotor_InfoCode 步进电机功能码
 * @ {
 */

// 只保留核心功能 其他信息码 如编码器值、输入脉冲数、目标位置、驱动温度、电池电压等信息 未写
#define STEPMOTOR_INFOCODE_ZEROSTATUS 0x3B        // 读取回零状态标志
#define STEPMOTOR_INFOCODE_HARDWAREVERSION 0x1F   // 读取硬件版本
#define STEPMOTOR_INFOCODE_CURRENTINDUCTANCE 0x20 // 读取相电流相电感
#define STEPMOTOR_INFOCODE_V_BUS 0x24             // 读取电机总线电压
#define STEPMOTOR_INFOCODE_CURRENT_BUS 0x26       // 读取电机总线电流
#define STEPMOTOR_INFOCODE_SPEED 0x35             // 读取电机速度
#define STEPMOTOR_INFOCODE_POSITION 0x36          // 读取电机位置
/**
 * @ }
 */

enum StepMotor_TurningMode
{
    StepMotor_TurningMode_RelateToLastTarget = 0, // 相对上一输入目标位置进行相对位置运动
    StepMotor_TurningMode_RelateToZero = 1,       // 相对坐标零点进行绝对位置运动
    StepMotor_TurningMode_RelateToRealPos = 2     // 相对当前实时位置进行相对位置运动
};

typedef struct
{
    uint32_t CanID; // 步进电机的CAN ID

    uint16_t dividerMode; // 细分模式 每次脉冲->转动 STEP_MOTOR_PULSE_ANGLE/DividerMode 度

    double currentSpeed; // 当前速度
    double currtAngle;   // 当前角度

    uint8_t dataBuf[16];   // 接收数据缓冲区
    uint16_t TurningAccel; // 转动加速度
    uint16_t TurningSpeed; // 转动速度

    uint16_t TurningMaxSpeed;   // 转动最大速度
    uint16_t TurningMaxCurrent; // 转动最大电流

} StepMotorStruct;

void StepMotor_ReturnMsgCallback(uint8_t rxData[], uint32_t CanID, uint32_t DataLength);

// 函数只进行指令的赋值与发送 内容参考官方说明书指令
// region 电机基础操作函数
void StepMotor_EncoderCal(StepMotorStruct *motor);
void StepMotor_Reset(StepMotorStruct *motor);
void StepMotor_ClearPosition(StepMotorStruct *motor);

void StepMotor_Enable(StepMotorStruct *motor);
void StepMotor_Disable(StepMotorStruct *motor);
// endregion 电机基础操作函数

// region 步进电机的复杂操作函数 未经过包装 不便使用
/** 步进电机的复杂操作函数
void StepMotor_SetForceMode_WithSpeedLimit_X(StepMotorStruct *motor, uint8_t motorDirect, uint16_t motorAccel, uint16_t motorCurrent, uint16_t motorMaxSpeed);

void StepMotor_SetSpeedMode_X(StepMotorStruct *motor, uint8_t motorDirect, uint16_t motorAccel, uint16_t motorSpeed);
void StepMotor_SetSpeedMode_WithCurrentLimit_X(StepMotorStruct *motor, uint8_t motorDirect, uint16_t motorAccel, uint16_t motorSpeed, uint16_t motorMaxCurrent);

void StepMotor_SetSpeedMode_Emm(StepMotorStruct *motor, uint8_t motorDirect, uint8_t motorAccel, uint16_t motorSpeed);

void StepMotor_SetPositionMode_X(StepMotorStruct *motor, uint8_t motorDirect, uint16_t motorSpeed, uint32_t motorTargetPosition, enum StepMotor_TurningMode turningMode);
void StepMotor_SetPositionMode_WithCurrentLimit_X(StepMotorStruct *motor, uint8_t motorDirect, uint16_t motorSpeed, uint32_t motorTargetPosition, enum StepMotor_TurningMode turningMode, uint16_t motorMaxCurrent);

void StepMotor_SetAccelerationMode_X(StepMotorStruct *motor, uint8_t motorDirect, uint16_t motorAccel, uint16_t motorDecel, uint16_t motorMaxSpeed, uint32_t motorTargetPosition, enum StepMotor_TurningMode turningMode);
void StepMotor_SetAccelerationMode_WithCurrentLimit_X(StepMotorStruct *motor, uint8_t motorDirect, uint16_t motorAccel, uint16_t motorDecel, uint16_t motorMaxSpeed, uint32_t motorTargetPosition, enum StepMotor_TurningMode turningMode, uint16_t motorMaxCurrent);

void StepMotor_SetPositionMode_Emm(StepMotorStruct *motor, uint8_t motorDirect, uint16_t motorSpeed, uint8_t motorAccel, double motorTargetPosition, enum StepMotor_TurningMode turningMode);

void StepMotor_EmergencyStop(StepMotorStruct *motor);
void StepMotor_EmergencyStop_All(void);

void StepMotor_SyncMove(void);
//------------------------------------------------

// 读取系统参数命令
void StepMotor_SetReturnInfo_Repeatly(StepMotorStruct *motor, uint8_t infoCode, uint16_t repeatTimes);
void StepMotor_AskForInfo(StepMotorStruct *motor, uint8_t infoCode);
//------------------------------------------------

// 读写驱动参数命令
// 建议在上位机中更改 不用代码实现
//------------------------------------------------

 */
// endregion 步进电机的复杂操作函数

// region 回零相关指令函数
void StepMotor_SetZero(StepMotorStruct *motor);
void StepMotor_BackToZero(StepMotorStruct *motor);
void StepMotor_EmergencyStop_BackToZero(StepMotorStruct *motor);

// 读取回零参数 未写
// 修改回零参数 未写
// endregion 回零相关指令函数

// 注意Emm固件和X固件在速度和位置表示上的不同
// 外部公开函数 外部通过以下函数进行操作

void StepMotor_Init(StepMotorStruct *motor, uint32_t motorAddr, uint16_t dividerMode);

void StepMotor_SetTurning(StepMotorStruct *motor, double motorSpeed);

void StepMotor_SetRelatAngle(StepMotorStruct *motor, double angle);

void StepMotor_SetAngle(StepMotorStruct *motor, double angle);

void StepMotor_Stop(StepMotorStruct *motor);

//------------------------------------------------

#endif // _CAN_CTRL_MODE_

#endif // __STEPMOTOR_H__
