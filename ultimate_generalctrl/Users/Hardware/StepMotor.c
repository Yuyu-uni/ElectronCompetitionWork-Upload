#include "StepMotor.h"

#ifdef _STEP_DIR_CTRL_MODE_
#include "math.h"

static StepMotorStruct *stepMotor[2];
/**
 * @brief 步进电机初始化函数
 * @param motor 步进电机结构体指针
 * @param motorTag 步进电机标识 StepMotorA或StepMotorB
 * @param dividerMode 步进电机细分模式
 * @note 该函数用于初始化步进电机的相关参数 默认当前电机已完成归零操作 转速为40转/分钟
 * @note 经过测试 细分模式仅8和128可用 1/8细分对应拨码开关0101  1/128细分对应拨码开关0000
 */
void StepMotor_Init(StepMotorStruct *motor, enum StepMotorTag motorTag, uint16_t dividerMode)
{
    /*回零操作或读取当前角度操作*/

    motor->currtAngle = 0.0; // 这种方式默认初始角度为0 请在每次上电完成回零程序

    //-----------------------
    stepMotor[motorTag] = motor; // 将步进电机结构体指针存储到全局数组中 以便在该函数中访问

    switch (motorTag)
    {
    case StepMotorA:
        motor->stepTimHandler = htim8;
        motor->stepTimChannel = TIM_CHANNEL_1; // 使用TIM8通道1
        motor->dirPin = StepA_DIR_Pin;
        motor->dirPinPort = StepA_DIR_GPIO_Port;
        break;
    case StepMotorB:
        motor->stepTimHandler = htim17;
        motor->stepTimChannel = TIM_CHANNEL_1; // 使用TIM17通道1
        motor->dirPin = StepB_DIR_Pin;
        motor->dirPinPort = StepB_DIR_GPIO_Port;
        break;

    default:
        break;
    }

    motor->dividerMode = dividerMode;
    motor->stepCounter = 0;
    motor->stepTurningFlag = 0;                                 // 初始状态未转动
    motor->anglePerStep = STEP_MOTOR_PULSE_ANGLE / dividerMode; // 每次脉冲转动的角度 单位为度

    HAL_GPIO_WritePin(motor->dirPinPort, motor->dirPin, GPIO_PIN_RESET); // 设置方向引脚为低电平
    StepMotor_SetSpeed(motor, 40);                                       // 设置速度为40
}

/*
 *@brief 设置步进电机的速度
 *@param motor 步进电机结构体指针
 *@param speed 步进电机的速度
 *@note 速度单位为  转每分钟 为保持带载扭矩和稳定性，建议在20-70转（未经测试）实测400以上会丢步。
 */
void StepMotor_SetSpeed(StepMotorStruct *motor, uint16_t speed)
{
    uint16_t PWMFreq = 6 * speed / motor->anglePerStep;
    uint16_t PWMPrescaler = 170000 / PWMFreq - 1; // 主频170MHz 自动重装载值为1000 Freq = 170M/Prescaler/1000

    __HAL_TIM_SET_PRESCALER(&motor->stepTimHandler, PWMPrescaler);
}

/**
 * @brief 使步进电机转动指定角度
 * @param motor 步进电机结构体指针
 * @param angle 需要转动的角度 单位为度 以右手定则 方向沿着电机轴方向向外 为正方向
 * @note 该函数使步进电机相对当前角度转动，请传入电机的步进角而非绝对角度
 */
void StepMotor_Turn(StepMotorStruct *motor, double angle)
{
    // 计算需要的步数（正负表示方向）
    int32_t steps = (int32_t)(angle / motor->anglePerStep);
    if (steps == 0)
    {
        return; // 如果步数为0，则不需要转动
    }

    // 设置方向
    if (steps >= 0)
    {
        HAL_GPIO_WritePin(motor->dirPinPort, motor->dirPin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(motor->dirPinPort, motor->dirPin, GPIO_PIN_RESET);
    }
    // 设置步进转动标志位
    motor->stepTurningFlag = 1;
    motor->stepCounter = steps;

    HAL_TIM_PWM_Start_IT(&motor->stepTimHandler, motor->stepTimChannel); // 启动PWM输出
}

/**
 * @brief 计算步进电机的转动 通过一次PWM脉冲上升沿对应一次步进角计算
 * @param motor 步进电机结构体指针
 * @note 该函数在PWM脉冲上升沿回调中调用
 */
void StepMotor_TurnCalc(StepMotorStruct *motor)
{
    if (motor->stepTurningFlag == 1) // 如果处于位置模式
    {
        if (motor->stepCounter > 0)
        {
            motor->stepCounter--;
            motor->currtAngle += motor->anglePerStep;           // 更新当前角度
            motor->currtAngle = fmod(motor->currtAngle, 360.0); // 保持角度在0-360度范围内
            if (motor->currtAngle < 0)
                motor->currtAngle += 360.0; // 确保角度为正值
        }
        else if (motor->stepCounter < 0)
        {
            motor->stepCounter++;
            motor->currtAngle -= motor->anglePerStep;           // 更新当前角度
            motor->currtAngle = fmod(motor->currtAngle, 360.0); // 保持角度在0-360度范围内
            if (motor->currtAngle < 0)
                motor->currtAngle += 360.0; // 确保角度为正值
        }
        else // 当步数为0时，表示转动完成
        {
            motor->stepTurningFlag = 0;                                         // 步进转动完成
            HAL_TIM_PWM_Stop_IT(&motor->stepTimHandler, motor->stepTimChannel); // 停止PWM输出
        }
    }
    else if ((motor->stepTurningFlag == 2)) // 如果处于定速转动模式
    {
        motor->currtAngle += motor->anglePerStep;           // 每次脉冲转动的角度 单位为度
        motor->currtAngle = fmod(motor->currtAngle, 360.0); // 保持角度在0-360度范围内
        if (motor->currtAngle < 0)
            motor->currtAngle += 360.0; // 确保角度为正值
    }
}

void StepMotor_Tick(enum StepMotorTag motorTag)
{
    StepMotor_TurnCalc(stepMotor[motorTag]);
}
/**
 * @brief 步进电机开始定速转动模式
 * @param motor 步进电机结构体指针
 * @param motorSpeed 转动速度 单位为度/秒
 */
void StepMotor_StartTurning(StepMotorStruct *motor, int16_t motorSpeed)
{
    if (motorSpeed < 0) // 如果速度为负数
    {
        HAL_GPIO_WritePin(motor->dirPinPort, motor->dirPin, GPIO_PIN_RESET); // 设置方向引脚为低电平
        motorSpeed = -motorSpeed;                                            // 转换为正数
    }
    else if (motorSpeed > 0) // 如果速度为正数
    {
        HAL_GPIO_WritePin(motor->dirPinPort, motor->dirPin, GPIO_PIN_SET); // 设置方向引脚为高电平
    }
    else
    {
        StepMotor_Stop(motor); // 如果速度为0，直接停止转动 防止预分频器趋于无穷大
        return;
    }
    StepMotor_SetSpeed(motor, motorSpeed);

    if (motor->stepTurningFlag == 2) // 如果已经处于定速转动模式
    {
        return; // 不需要重复启动
    }
    else
    {
        motor->stepTurningFlag = 2;                                          // 设置为定速转动模式
        motor->stepCounter = 0;                                              // 重置步进计数器
        HAL_TIM_PWM_Start_IT(&motor->stepTimHandler, motor->stepTimChannel); // 启动PWM输出
    }
}
/**
 * @brief 步进电机停止转动
 * @param motor 步进电机结构体指针
 * @param motorSpeed 转动速度 单位为度/秒
 * @note 该函数用于停止步进电机的转动，退出转动模式 在定速转动模式改为位置模式时推荐调用此函数 防止未知问题（其实不调用应该也没啥）
 */
void StepMotor_Stop(StepMotorStruct *motor)
{
    motor->stepTurningFlag = 0;                                         // 停止转动
    motor->stepCounter = 0;                                             // 重置步进计数器
    HAL_TIM_PWM_Stop_IT(&motor->stepTimHandler, motor->stepTimChannel); // 停止PWM输出
}
/**
 * @brief 设置步进电机的目标角度
 * @param motor 步进电机结构体指针
 * @param angle 目标角度 范围为0---360度 最小分度值与细分模式相关 1.8°/8 = 0.225°/step | 1.8°/128 = 0.0140625°/step
 */
void StepMotor_SetAngle(StepMotorStruct *motor, double angle)
{
    // 保持角度在0-360度范围内
    angle = fmod(angle, 360.0);
    if (angle > 180.0)
    {
        angle -= 360.0; // 确保角度为负值 保证从最近路径转动
    }

    // 计算需要转动的角度
    double angleToTurn = angle - motor->currtAngle;

    // 调用转动函数
    StepMotor_Turn(motor, angleToTurn);
}

#endif

#ifdef _CAN_CTRL_MODE_
#include <string.h>

static StepMotorStruct *stepMotor[StepMotorNum];
static uint8_t StepMotorIndex = 0;

/**
 * @brief 该函数向FDCAN1的接收回调函数请求数据 在FDCAN1接收到消息时被调用
 * @param rxData 接收的数据缓冲区
 * @param CanID 发送消息者的CAN ID
 * @note 该函数将接收到的数据进行解析 并赋值给相应的步进电机的相应参数
 */
void StepMotor_ReturnMsgCallback(uint8_t rxData[], uint32_t CanID, uint32_t DataLength)
{
    uint8_t i;
    for (i = 0; i < StepMotorNum; i++)
    {
        if (stepMotor[i]->CanID == (CanID & 0xFFFFFF00))
        {
            switch (CanID & 0x000000FF) // 取低八位作为分包符
            {
            case 0x00:
                memcpy(stepMotor[i]->dataBuf, rxData, DataLength);
                break;
            case 0x01:
                memcpy(stepMotor[i]->dataBuf + 8, rxData, DataLength);
                break;
            default:
                break;
            }
            break;
        }
    }
    // 跳出循环，已确定对应的步进电机ID 进行下一步解析
    //  注意Emm固件和X固件在速度和位置表示上的不同

#ifdef _CAN_CTRL_FRAMEVER_X

    switch (stepMotor[i]->dataBuf[0]) // 根据数据包的功能码进行解析
    {
    case STEPMOTOR_INFOCODE_ZEROSTATUS:       // 读取回零状态标志
        if (stepMotor[i]->dataBuf[9] == 0x6B) // 确保完全接收
        {

            memset(stepMotor[i]->dataBuf, 0, sizeof(stepMotor[i]->dataBuf)); // 清空数据缓冲区
        }

        break;
    case STEPMOTOR_INFOCODE_SPEED:            // 读取电机速度
        if (stepMotor[i]->dataBuf[4] == 0x6B) // 确保完全接收
        {
            stepMotor[i]->currentSpeed = ((double)((stepMotor[i]->dataBuf[2] << 8) | stepMotor[i]->dataBuf[3])) / 10.0; // 解析速度值
            if (stepMotor[i]->dataBuf[1] == 0x01)                                                                       // 如果速度值为负数
            {
                stepMotor[i]->currentSpeed = -stepMotor[i]->currentSpeed; // 转换为负值
            }
            memset(stepMotor[i]->dataBuf, 0, sizeof(stepMotor[i]->dataBuf)); // 清空数据缓冲区
        }

        break;
    case STEPMOTOR_INFOCODE_POSITION: // 读取电机位置
        if (stepMotor[i]->dataBuf[6] == 0x6B)
        {
            stepMotor[i]->currtAngle = ((double)((stepMotor[i]->dataBuf[2] << 24) | (stepMotor[i]->dataBuf[3] << 16) | (stepMotor[i]->dataBuf[4] << 8) | stepMotor[i]->dataBuf[5])) / 10.0;
            if (stepMotor[i]->dataBuf[1] == 0x01)
            {
                stepMotor[i]->currtAngle = -stepMotor[i]->currtAngle;
            }
            memset(stepMotor[i]->dataBuf, 0, sizeof(stepMotor[i]->dataBuf)); // 清空数据缓冲区
        }

        break;
    default:
        break;
    }

#endif // X

#ifdef _CAN_CTRL_FRAMEVER_Emm
    switch (stepMotor[i]->dataBuf[0]) // 根据数据包的功能码进行解析
    {
    case STEPMOTOR_INFOCODE_ZEROSTATUS:       // 读取回零状态标志
        if (stepMotor[i]->dataBuf[9] == 0x6B) // 确保完全接收
        {

            memset(stepMotor[i]->dataBuf, 0, sizeof(stepMotor[i]->dataBuf)); // 清空数据缓冲区
        }

        break;
    case STEPMOTOR_INFOCODE_SPEED:            // 读取电机速度
        if (stepMotor[i]->dataBuf[4] == 0x6B) // 确保完全接收
        {
            stepMotor[i]->currentSpeed = (double)((stepMotor[i]->dataBuf[2] << 8) | stepMotor[i]->dataBuf[3]); // 解析速度值
            if (stepMotor[i]->dataBuf[1] == 0x01)                                                              // 如果速度值为负数
            {
                stepMotor[i]->currentSpeed = -stepMotor[i]->currentSpeed; // 转换为负值
            }
            memset(stepMotor[i]->dataBuf, 0, sizeof(stepMotor[i]->dataBuf)); // 清空数据缓冲区
        }

        break;
    case STEPMOTOR_INFOCODE_POSITION: // 读取电机位置
        if (stepMotor[i]->dataBuf[6] == 0x6B)
        {
            stepMotor[i]->currtAngle = (double)((stepMotor[i]->dataBuf[2] << 24) | (stepMotor[i]->dataBuf[3] << 16) | (stepMotor[i]->dataBuf[4] << 8) | stepMotor[i]->dataBuf[5]);
            stepMotor[i]->currtAngle = (stepMotor[i]->currtAngle * 360.0) / 65536.0;
            if (stepMotor[i]->dataBuf[1] == 0x01)
            {
                stepMotor[i]->currtAngle = -stepMotor[i]->currtAngle;
            }
            memset(stepMotor[i]->dataBuf, 0, sizeof(stepMotor[i]->dataBuf)); // 清空数据缓冲区
        }
        break;
    default:
        break;
    }

#endif // Emm
}

/**
 * @brief 步进电机编码器校准函数
 * @param motor 步进电机结构体指针
 * @note 该函数用于发送编码器校准指令
 * @note 闭环模式触发校准编码器，电机将缓慢正转一圈然后反转一圈对编码器进行线性化校准，提高精度
 */
void StepMotor_EncoderCal(StepMotorStruct *motor)
{
    uint8_t txDataBuf[3] = {0x06, 0x45, 0x6B}; // 编码器校准指令
    FDCAN1_AddMessageToTxFifo(txDataBuf, motor->CanID, FDCAN_DLC_BYTES_3);
}
/**
 * @brief 步进电机复位函数
 * @param motor 步进电机结构体指针
 * @note 该函数用于发送复位指令 电机将复位重启
 * @note 如果有接入“掉电记录”电池，复位重启电机位置不丢失
 */
void StepMotor_Reset(StepMotorStruct *motor)
{
    uint8_t txDataBuf[3] = {0x08, 0x97, 0x6B}; // 电机复位指令
    FDCAN1_AddMessageToTxFifo(txDataBuf, motor->CanID, FDCAN_DLC_BYTES_3);
}
/**
 * @brief 步进电机清除位置函数
 * @param motor 步进电机结构体指针
 * @note 该函数用于发送清除位置指令 电机将清除当前角度位置
 * @note 清除后电机的当前角度将被重置为0
 */
void StepMotor_ClearPosition(StepMotorStruct *motor)
{
    uint8_t txDataBuf[3] = {0x0A, 0x6D, 0x6B}; // 电机清除位置指令
    FDCAN1_AddMessageToTxFifo(txDataBuf, motor->CanID, FDCAN_DLC_BYTES_3);
}
/**
 * @brief 步进电机使能函数
 * @param motor 步进电机结构体指针
 * @note 该函数用于发送使能指令 电机将开始工作
 * @note 不使能电机会松轴，轴能拧动
 */
void StepMotor_Enable(StepMotorStruct *motor)
{
    uint8_t txDataBuf[5] = {0xF3, 0xAB, 0x01, 0x00, 0x6B}; // 电机使能指令
    FDCAN1_AddMessageToTxFifo(txDataBuf, motor->CanID, FDCAN_DLC_BYTES_5);
}
/**
 * @brief 步进电机禁用函数
 * @param motor 步进电机结构体指针
 * @note 该函数用于发送禁用指令 电机将停止工作
 * @note 禁用后电机将松轴，轴能拧动
 */
void StepMotor_Disable(StepMotorStruct *motor)
{
    uint8_t txDataBuf[5] = {0xF3, 0xAB, 0x00, 0x00, 0x6B}; // 电机禁用指令
    FDCAN1_AddMessageToTxFifo(txDataBuf, motor->CanID, FDCAN_DLC_BYTES_5);
}

/**
 * @brief 设置步进电机力矩限速控制模式
 * @param motor 步进电机结构体指针
 * @param motorDirect 电机转动方向 STEPMOTOR_DIRECT_FORWARD（逆时针）或STEPMOTOR_DIRECT_BACKWARD（顺时针）
 * @param motorAccel 电机加速度 [0-65535] 单位为毫安每秒
 * @param motorCurrent 电机电流 [0-5000mA] 单位为毫安（根据电机控制理论 电机电流可以影响电机转速和电机力矩 通过控制输入电流 可以控制电机力矩）
 * @param motorMaxSpeed 电机最大速度 [0-30000，保留一位小数，即0-3000.0RPM] 单位为0.1度/秒 驱动器会控制电机小于该速度工作
 * @note ### 该函数设置步进电机的力矩模式限速控制（仅适用于X固件）
 * @note -----
 * @note - 力矩模式控制，电机将会以给定电流和给定最大速度的最小值进行一直旋转，
 * @note - 如果给定电流太大，电机将会以给定最大速度运行，运行过程中将达不到给定电流，
 * @note 而是在夹爪夹紧(堵住)后达到给定电流，并维持给定电流力矩，适合电动夹爪、自动收线等应用。
 */
void StepMotor_SetForceMode_WithSpeedLimit_X(StepMotorStruct *motor, uint8_t motorDirect, uint16_t motorAccel, uint16_t motorCurrent, uint16_t motorMaxSpeed)
{
    uint8_t txDataBuf1[8];
    txDataBuf1[0] = 0xC5;                        // 功能码
    txDataBuf1[1] = motorDirect;                 // 方向
    txDataBuf1[2] = (motorAccel >> 8) & 0xFF;    // 高字节
    txDataBuf1[3] = motorAccel & 0xFF;           // 低字节
    txDataBuf1[4] = (motorCurrent >> 8) & 0xFF;  // 高字节
    txDataBuf1[5] = motorCurrent & 0xFF;         // 低字节
    txDataBuf1[6] = 0x00;                        // 同步标志
    txDataBuf1[7] = (motorMaxSpeed >> 8) & 0xFF; // 最大速度高字节

    FDCAN1_AddMessageToTxFifo(txDataBuf1, (motor->CanID), FDCAN_DLC_BYTES_8);

    // 分包发送
    uint8_t txDataBuf2[3];
    txDataBuf2[0] = 0xC5;                 // 功能码
    txDataBuf2[1] = motorMaxSpeed & 0xFF; // 最大速度低字节
    txDataBuf2[2] = 0x6B;                 // 校验位
    FDCAN1_AddMessageToTxFifo(txDataBuf2, (motor->CanID) | 0x01, FDCAN_DLC_BYTES_3);
}
/**
 * @brief 设置步进电机速度模式控制
 * @param motor 步进电机结构体指针
 * @param motorDirect 电机转动方向 STEPMOTOR_DIRECT_FORWARD（逆时针）或STEPMOTOR_DIRECT_BACKWARD（顺时针）
 * @param motorAccel 电机加速度 [0-65535] 单位为RPM/S
 * @param motorSpeed 电机速度 [0-30000，保留一位小数，即0-3000.0RPM] 单位为0.1RPM
 * @note ### 该函数设置步进电机的速度模式控制（仅适用于X固件）
 * @note -----
 * @note - 速度模式控制，电机将会以给定速度一直旋转
 * @note - 注意：此模式无最大电流限制
 */
void StepMotor_SetSpeedMode_X(StepMotorStruct *motor, uint8_t motorDirect, uint16_t motorAccel, uint16_t motorSpeed)
{
    uint8_t txDataBuf1[8];
    txDataBuf1[0] = 0xF6;                     // 功能码
    txDataBuf1[1] = motorDirect;              // 方向
    txDataBuf1[2] = (motorAccel >> 8) & 0xFF; // 高字节
    txDataBuf1[3] = motorAccel & 0xFF;        // 低字节
    txDataBuf1[4] = (motorSpeed >> 8) & 0xFF; // 高字节
    txDataBuf1[5] = motorSpeed & 0xFF;        // 低字节
    txDataBuf1[6] = 0x00;                     // 同步标志
    txDataBuf1[7] = 0x6B;                     // 校验位

    FDCAN1_AddMessageToTxFifo(txDataBuf1, (motor->CanID), FDCAN_DLC_BYTES_8);
}
/**
 * @brief 设置步进电机速度模式控制并带有电流限制
 * @param motor 步进电机结构体指针
 * @param motorDirect 电机转动方向 STEPMOTOR_DIRECT_FORWARD（逆时针）或STEPMOTOR_DIRECT_BACKWARD（顺时针）
 * @param motorAccel 电机加速度 [0-65535] 单位为RPM/S
 * @param motorSpeed 电机速度 [0-30000，保留一位小数，即0-3000.0RPM] 单位为0.1RPM
 * @param motorMaxCurrent 电机最大电流 [0-5000mA] 单位为毫安
 * @note ### 该函数设置步进电机的速度模式控制并带有电流限制（仅适用于X固件）
 * @note -----
 * @note - 速度模式控制，电机将会以给定速度一直旋转
 * @note - 注意：此模式有最大电流限制
 */
void StepMotor_SetSpeedMode_WithCurrentLimit_X(StepMotorStruct *motor, uint8_t motorDirect, uint16_t motorAccel, uint16_t motorSpeed, uint16_t motorMaxCurrent)
{
    uint8_t txDataBuf1[8];
    txDataBuf1[0] = 0xC6;                          // 功能码
    txDataBuf1[1] = motorDirect;                   // 方向
    txDataBuf1[2] = (motorAccel >> 8) & 0xFF;      // 高字节
    txDataBuf1[3] = motorAccel & 0xFF;             // 低字节
    txDataBuf1[4] = (motorSpeed >> 8) & 0xFF;      // 高字节
    txDataBuf1[5] = motorSpeed & 0xFF;             // 低字节
    txDataBuf1[6] = 0x00;                          // 同步标志
    txDataBuf1[7] = (motorMaxCurrent >> 8) & 0xFF; // 最大电流高字节

    FDCAN1_AddMessageToTxFifo(txDataBuf1, (motor->CanID), FDCAN_DLC_BYTES_8);

    // 分包发送
    uint8_t txDataBuf2[3];
    txDataBuf2[0] = 0xC6;                   // 功能码
    txDataBuf2[1] = motorMaxCurrent & 0xFF; // 最大电流低字节
    txDataBuf2[2] = 0x6B;                   // 校验位
    FDCAN1_AddMessageToTxFifo(txDataBuf2, (motor->CanID) | 0x01, FDCAN_DLC_BYTES_3);
}
/**
 * @brief 设置步进电机速度模式控制（Emm固件）
 * @param motor 步进电机结构体指针
 * @param motorDirect 电机转动方向 STEPMOTOR_DIRECT_FORWARD（逆时针）或STEPMOTOR_DIRECT_BACKWARD（顺时针）
 * @param motorAccel 电机加速度 [0-255] 单位为RPM/S
 * @param motorSpeed 电机速度 [0-3000] 单位为RPM
 * @note ### 该函数设置步进电机的速度模式控制（仅适用于Emm固件）
 * @note -----
 * @note - 速度模式控制，电机将会以给定速度和给定加速度一直旋转
 * @note -----
 * @note - 加速设置说明：
 * @note 1. 加速度为0表示不使用曲线加减速，直接按照设定的速度启动和运行。
 * @note 2. 加速度不为0，则曲线加减速时间计算公式为: t2 - t1 = (256 - acc) * 50(us)，Vt2 = Vt1 + 1(RPM);
 */
void StepMotor_SetSpeedMode_Emm(StepMotorStruct *motor, uint8_t motorDirect, uint8_t motorAccel, uint16_t motorSpeed)
{
    uint8_t txDataBuf[7];

    txDataBuf[0] = 0xF6;              // 功能码
    txDataBuf[1] = motorDirect;       // 方向
    txDataBuf[2] = (motorSpeed >> 8); // 高字节
    txDataBuf[3] = motorSpeed & 0xFF; // 低字节
    txDataBuf[4] = motorAccel;        // 加速度
    txDataBuf[5] = 0x00;              // 同步标志
    txDataBuf[6] = 0x6B;              // 校验位

    FDCAN1_AddMessageToTxFifo(txDataBuf, (motor->CanID), FDCAN_DLC_BYTES_7);
}
/**
 * @brief 设置步进电机位置模式控制
 * @param motor 步进电机结构体指针
 * @param motorDirect 电机转动方向 STEPMOTOR_DIRECT_FORWARD（逆时针）或STEPMOTOR_DIRECT_BACKWARD（顺时针）
 * @param motorSpeed 电机速度 [0-30000，保留一位小数，即0-3000.0RPM] 单位为0.1RPM
 * @param motorTargetPosition 目标位置 [0-4294967295] 单位为 0.1度
 * @param turningMode 转动模式 参考 StepMotor_TurningMode 枚举类型
 * @note ### 该函数设置步进电机的位置模式控制（仅适用于X固件）
 * @note -----
 * @note - 位置模式控制，电机将会以给定速度和给定加速度转动到指定位置
 * @note - 转动模式说明：
 * @note 1. StepMotor_TurningMode_RelateToLastTarget：相对上一输入目标位置进行相对位置运动
 * @note 2. StepMotor_TurningMode_RelateToZero：相对坐标零点进行绝对位置运动
 * @note 3. StepMotor_TurningMode_RelateToRealPos：相对当前实时位置进行相对位置运动
 */
void StepMotor_SetPositionMode_X(StepMotorStruct *motor, uint8_t motorDirect, uint16_t motorSpeed, uint32_t motorTargetPosition, enum StepMotor_TurningMode turningMode)
{
    uint8_t txDataBuf1[8];

    txDataBuf1[0] = 0xFB;                               // 功能码
    txDataBuf1[1] = motorDirect;                        // 方向
    txDataBuf1[2] = (motorSpeed >> 8) & 0xFF;           // 高字节
    txDataBuf1[3] = motorSpeed & 0xFF;                  // 低字节
    txDataBuf1[4] = (motorTargetPosition >> 24) & 0xFF; // 目标位置高字节
    txDataBuf1[5] = (motorTargetPosition >> 16) & 0xFF; // 目标位置次高字节
    txDataBuf1[6] = (motorTargetPosition >> 8) & 0xFF;  // 目标位置次低字节
    txDataBuf1[7] = motorTargetPosition & 0xFF;         // 目标位置低字节

    FDCAN1_AddMessageToTxFifo(txDataBuf1, (motor->CanID), FDCAN_DLC_BYTES_8);

    // 分包发送
    uint8_t txDataBuf2[4];
    txDataBuf2[0] = 0xFB;        // 功能码
    txDataBuf2[1] = turningMode; // 转动模式
    txDataBuf2[2] = 0x00;        // 同步标志
    txDataBuf2[3] = 0x6B;        // 校验位
    FDCAN1_AddMessageToTxFifo(txDataBuf2, (motor->CanID) | 0x01, FDCAN_DLC_BYTES_4);
}
/**
 * @brief 设置步进电机位置模式控制并带有电流限制
 * @param motor 步进电机结构体指针
 * @param motorDirect 电机转动方向 STEPMOTOR_DIRECT_FORWARD（逆时针）或STEPMOTOR_DIRECT_BACKWARD（顺时针）
 * @param motorSpeed 电机速度 [0-30000，保留一位小数，即0-3000.0RPM] 单位为0.1RPM
 * @param motorTargetPosition 目标位置 [0-4294967295] 单位为 0.1度
 * @param turningMode 转动模式 参考 StepMotor_TurningMode 枚举类型
 * @param motorMaxCurrent 电机最大电流 [0-5000mA] 单位为毫安
 * @note ### 该函数设置步进电机的位置模式控制并带有电流限制（仅适用于X固件）
 * @note -----
 * @note - 位置模式控制，电机将会以给定速度和给定加速度转动到指定位置
 * @note - 转动模式说明：
 * @note 1. StepMotor_TurningMode_RelateToLastTarget：相对上一输入目标位置进行相对位置运动
 * @note 2. StepMotor_TurningMode_RelateToZero：相对坐标零点进行绝对位置运动
 * @note 3. StepMotor_TurningMode_RelateToRealPos：相对当前实时位置进行相对位置运动
 */
void StepMotor_SetPositionMode_WithCurrentLimit_X(StepMotorStruct *motor, uint8_t motorDirect, uint16_t motorSpeed, uint32_t motorTargetPosition, enum StepMotor_TurningMode turningMode, uint16_t motorMaxCurrent)
{
    uint8_t txDataBuf1[8];
    txDataBuf1[0] = 0xCB;                               // 功能码
    txDataBuf1[1] = motorDirect;                        // 方向
    txDataBuf1[2] = (motorSpeed >> 8) & 0xFF;           // 高字节
    txDataBuf1[3] = motorSpeed & 0xFF;                  //
    txDataBuf1[4] = (motorTargetPosition >> 24) & 0xFF; // 目标位置高字节
    txDataBuf1[5] = (motorTargetPosition >> 16) & 0xFF; // 目标位置次高字节
    txDataBuf1[6] = (motorTargetPosition >> 8) & 0xFF;  // 目标位置次低字节
    txDataBuf1[7] = motorTargetPosition & 0xFF;         // 目标位置低字节
    FDCAN1_AddMessageToTxFifo(txDataBuf1, (motor->CanID), FDCAN_DLC_BYTES_8);
    // 分包发送
    uint8_t txDataBuf2[6];
    txDataBuf2[0] = 0xCB;                          // 功能码
    txDataBuf2[1] = turningMode;                   // 转动模式
    txDataBuf2[2] = 0x00;                          // 同步标志
    txDataBuf2[3] = (motorMaxCurrent >> 8) & 0xFF; // 最大电流高字节
    txDataBuf2[4] = motorMaxCurrent & 0xFF;        // 最大电流低字节
    txDataBuf2[5] = 0x6B;                          // 校验位
    FDCAN1_AddMessageToTxFifo(txDataBuf2, (motor->CanID) | 0x01, FDCAN_DLC_BYTES_6);
}
/**
 * @brief 设置步进电机加速度位置控制模式
 * @param motor 步进电机结构体指针
 * @param motorDirect 电机转动方向 STEPMOTOR_DIRECT_FORWARD（逆时针）或STEPMOTOR_DIRECT_BACKWARD（顺时针）
 * @param motorAccel 电机加速度 [0-65535] 单位为RPM/S
 * @param motorDecel 电机减速度 [0-65535] 单位为RPM/S
 * @param motorMaxSpeed 电机最大速度 [0-30000，保留一位小数，即0-3000.0RPM] 单位为0.1RPM
 * @param motorTargetPosition 目标位置 [0-4294967295] 单位为 0.1度
 * @param turningMode 转动模式 参考 StepMotor_TurningMode 枚举类型
 * @note ### 该函数设置步进电机的加速度模式控制（仅适用于X固件）
 * @note -----
 * @note - 加速度模式控制，电机将会以**给定加速度**达到 最大速度 并且在最后通过**给定减速度**转动到指定位置
 * @note 转动参考案例链接（有速度图像辅助理解）[官方说明书参考案例链接](https://www.yuque.com/zhangdatouzhikong/gzng3d/kpy15dvfda1gykc6)
 * @note - 转动模式说明：
 * @note 1. StepMotor_TurningMode_RelateToLastTarget：相对上一输入目标位置进行相对位置运动
 * @note 2. StepMotor_TurningMode_RelateToZero：相对坐标零点进行绝对位置运动
 * @note 3. StepMotor_TurningMode_RelateToRealPos：相对当前实时位置进行相对位置运动
 */
void StepMotor_SetAccelerationMode_X(StepMotorStruct *motor, uint8_t motorDirect, uint16_t motorAccel, uint16_t motorDecel, uint16_t motorMaxSpeed, uint32_t motorTargetPosition, enum StepMotor_TurningMode turningMode)
{
    uint8_t txDataBuf1[8];

    txDataBuf1[0] = 0xFD;                        // 功能码
    txDataBuf1[1] = motorDirect;                 // 方向
    txDataBuf1[2] = (motorAccel >> 8) & 0xFF;    // 高字节
    txDataBuf1[3] = motorAccel & 0xFF;           // 低字节
    txDataBuf1[4] = (motorDecel >> 8) & 0xFF;    // 高字节
    txDataBuf1[5] = motorDecel & 0xFF;           // 低字节
    txDataBuf1[6] = (motorMaxSpeed >> 8) & 0xFF; // 最大速度高字节
    txDataBuf1[7] = motorMaxSpeed & 0xFF;        // 最大速度低字节
    FDCAN1_AddMessageToTxFifo(txDataBuf1, (motor->CanID), FDCAN_DLC_BYTES_8);

    // 分包发送
    uint8_t txDataBuf2[8];
    txDataBuf2[0] = 0xFD;                               // 功能码
    txDataBuf2[1] = (motorTargetPosition >> 24) & 0xFF; // 目标位置高字节
    txDataBuf2[2] = (motorTargetPosition >> 16) & 0xFF; // 目标位置次高字节
    txDataBuf2[3] = (motorTargetPosition >> 8) & 0xFF;  // 目标位置次低字节
    txDataBuf2[4] = motorTargetPosition & 0xFF;         // 目标位置低字节
    txDataBuf2[5] = turningMode;                        // 转动模式
    txDataBuf2[6] = 0x00;                               // 同步标志
    txDataBuf2[7] = 0x6B;                               // 校验位
    FDCAN1_AddMessageToTxFifo(txDataBuf2, (motor->CanID) | 0x01, FDCAN_DLC_BYTES_8);
}
/**
 * @brief 设置步进电机加速度位置控制模式并带有电流限制
 * @param motor 步进电机结构体指针
 * @param motorDirect 电机转动方向 STEPMOTOR_DIRECT_FORWARD（逆时针）或STEPMOTOR_DIRECT_BACKWARD（顺时针）
 * @param motorAccel 电机加速度 [0-65535] 单位为RPM/S
 * @param motorDecel 电机减速度 [0-65535] 单位为RPM/S
 * @param motorMaxSpeed 电机最大速度 [0-30000，保留一位小数，即0-3000.0RPM] 单位为0.1RPM
 * @param motorTargetPosition 目标位置 [0-4294967295] 单位为 0.1度
 * @param turningMode 转动模式 参考 StepMotor_TurningMode 枚举类型
 * @param motorMaxCurrent 电机最大电流 [0-5000mA] 单位为毫安
 * @note ### 该函数设置步进电机的加速度模式控制并带有电流限制（仅适用于X固件）
 * @note -----
 * @note - 加速度模式控制，电机将会以**给定加速度**达到 最大速度 并且在最后通过**给定减速度**转动到指定位置 过程中控制电流不超过最大电流
 * @note 转动参考案例链接（有速度图像辅助理解）[官方说明书参考案例链接](https://www.yuque.com/zhangdatouzhikong/gzng3d/kpy15dvfda1gykc6)
 * @note - 转动模式说明：
 * @note 1. StepMotor_TurningMode_RelateToLastTarget：相对上一输入目标位置进行相对位置运动
 * @note 2. StepMotor_TurningMode_RelateToZero：相对坐标零点进行绝对位置运动
 * @note 3. StepMotor_TurningMode_RelateToRealPos：相对当前实时位置进行相对位置运动
 */
void StepMotor_SetAccelerationMode_WithCurrentLimit_X(StepMotorStruct *motor, uint8_t motorDirect, uint16_t motorAccel, uint16_t motorDecel, uint16_t motorMaxSpeed, uint32_t motorTargetPosition, enum StepMotor_TurningMode turningMode, uint16_t motorMaxCurrent)
{
    uint8_t txDataBuf1[8];

    txDataBuf1[0] = 0xCD;                        // 功能码
    txDataBuf1[1] = motorDirect;                 // 方向
    txDataBuf1[2] = (motorAccel >> 8) & 0xFF;    // 高字节
    txDataBuf1[3] = motorAccel & 0xFF;           // 低字节
    txDataBuf1[4] = (motorDecel >> 8) & 0xFF;    // 高字节
    txDataBuf1[5] = motorDecel & 0xFF;           // 低字节
    txDataBuf1[6] = (motorMaxSpeed >> 8) & 0xFF; // 最大速度高字节
    txDataBuf1[7] = motorMaxSpeed & 0xFF;        // 最大速度低字节
    FDCAN1_AddMessageToTxFifo(txDataBuf1, (motor->CanID), FDCAN_DLC_BYTES_8);

    // 分包发送
    uint8_t txDataBuf2[8];
    txDataBuf2[0] = 0xCD;                               // 功能码
    txDataBuf2[1] = (motorTargetPosition >> 24) & 0xFF; // 目标位置高字节
    txDataBuf2[2] = (motorTargetPosition >> 16) & 0xFF; // 目标位置次高字节
    txDataBuf2[3] = (motorTargetPosition >> 8) & 0xFF;  // 目标位置次低字节
    txDataBuf2[4] = motorTargetPosition & 0xFF;         // 目标位置低字节
    txDataBuf2[5] = turningMode;                        // 转动模式
    txDataBuf2[6] = 0x00;                               // 同步标志
    txDataBuf2[7] = (motorMaxCurrent >> 8) & 0xFF;      // 最大电流高字节
    FDCAN1_AddMessageToTxFifo(txDataBuf2, (motor->CanID) | 0x01, FDCAN_DLC_BYTES_8);

    uint8_t txDataBuf3[3];
    txDataBuf3[0] = 0xCD;                   // 功能码
    txDataBuf3[1] = motorMaxCurrent & 0xFF; // 最大电流低字节
    txDataBuf3[2] = 0x6B;                   // 校验位

    FDCAN1_AddMessageToTxFifo(txDataBuf3, (motor->CanID) | 0x02, FDCAN_DLC_BYTES_3);
}
/**
 * @brief 设置步进电机位置模式控制（Emm固件）
 * @param motor 步进电机结构体指针
 * @param motorDirect 电机转动方向 STEPMOTOR_DIRECT_FORWARD（逆时针）或STEPMOTOR_DIRECT_BACKWARD（顺时针）
 * @param motorSpeed 电机速度 [0-3000] 单位为RPM
 * @param motorAccel 电机加速度 [0-255] 单位为RPM/S
 * @param motorTargetPosition 目标位置 [0-4294967295] 单位为0.1度 （为了和X固件模式统一）
 * @param turningMode 转动模式 参考 StepMotor_TurningMode 枚举类型
 * @note ### 该函数设置步进电机的位置模式控制（仅适用于Emm固件）
 * @note -----
 * @note - 位置模式控制，电机将会以给定速度和给定加速度转动到指定位置
 * @note - 转动模式说明：
 * @note 1. StepMotor_TurningMode_RelateToLastTarget：相对上一输入目标位置进行相对位置运动
 * @note 2. StepMotor_TurningMode_RelateToZero：相对坐标零点进行绝对位置运动
 * @note 3. StepMotor_TurningMode_RelateToRealPos：相对当前实时位置进行相对位置运动
 */
void StepMotor_SetPositionMode_Emm(StepMotorStruct *motor, uint8_t motorDirect, uint16_t motorSpeed, uint8_t motorAccel, double motorTargetPosition, enum StepMotor_TurningMode turningMode)
{
    uint32_t motorTargetPosition_Pulse = (uint32_t)(motorTargetPosition * motor->dividerMode / STEP_MOTOR_PULSE_ANGLE / 10.0); // 转换为脉冲数
    uint8_t txDataBuf1[8];
    txDataBuf1[0] = 0xFD;                                     // 功能码
    txDataBuf1[1] = motorDirect;                              // 方向
    txDataBuf1[2] = (motorSpeed >> 8);                        // 高字节
    txDataBuf1[3] = motorSpeed & 0xFF;                        // 低字节
    txDataBuf1[4] = motorAccel;                               // 加速度
    txDataBuf1[5] = (motorTargetPosition_Pulse >> 24) & 0xFF; // 目标位置高字节
    txDataBuf1[6] = (motorTargetPosition_Pulse >> 16) & 0xFF; // 目标位置次高字节
    txDataBuf1[7] = (motorTargetPosition_Pulse >> 8) & 0xFF;  // 目标位置次低字节

    FDCAN1_AddMessageToTxFifo(txDataBuf1, (motor->CanID), FDCAN_DLC_BYTES_8);
    // 分包发送
    uint8_t txDataBuf2[5];
    txDataBuf2[0] = 0xFD;                             // 功能码
    txDataBuf2[1] = motorTargetPosition_Pulse & 0xFF; // 目标位置低字节
    txDataBuf2[2] = turningMode;                      // 转动模式
    txDataBuf2[3] = 0x00;                             // 同步标志
    txDataBuf2[4] = 0x6B;                             // 校验位
    FDCAN1_AddMessageToTxFifo(txDataBuf2, (motor->CanID) | 0x01, FDCAN_DLC_BYTES_5);
}
/**
 * @brief 步进电机紧急停止函数
 * @param motor 步进电机结构体指针
 * @note 该函数用于发送紧急停止指令 电机将立即停止工作
 */
void StepMotor_EmergencyStop(StepMotorStruct *motor)
{
    uint8_t txDataBuf[4] = {0xFE, 0x98, 0x00, 0x6B}; // 紧急停止指令
    FDCAN1_AddMessageToTxFifo(txDataBuf, (motor->CanID), FDCAN_DLC_BYTES_4);
}
void StepMotor_EmergencyStop_All(void)
{
    uint8_t txDataBuf[4] = {0xFE, 0x98, 0x00, 0x6B}; // 紧急停止指令
    FDCAN1_AddMessageToTxFifo(txDataBuf, ALL_STEPMOTOR_CTRL, FDCAN_DLC_BYTES_4);
}
/**
 * @brief 步进电机同步移动函数
 * @note ### 该函数用于发送同步开始移动指令 所有步进电机将同时开始移动
 * @note -----
 * @note 以广播地址0发送多机同步运动命令，将使缓存了命令的电机(发送过同步标志为01的命令)同步开始执行缓存的命令，无缓存命令的电机不会动作
 */
void StepMotor_SyncMove(void)
{
    uint8_t txDataBuf[3] = {0xFF, 0x66, 0x6B}; // 同步开始移动指令
    FDCAN1_AddMessageToTxFifo(txDataBuf, ALL_STEPMOTOR_CTRL, FDCAN_DLC_BYTES_3);
}
/**
 * @brief 步进电机设置零点函数
 * @param motor 步进电机结构体指针
 * @note 该函数用于发送设置零点指令 电机将当前角度位置设置为参考零点 默认电机**储存**当前设置的零点位置 第三位是1表示储存当前位置，数据掉电不丢失[0x00-->零点位置不储存]
 * @note ### 注意区分坐标零点与回零零点
 */
void StepMotor_SetZero(StepMotorStruct *motor)
{
    uint8_t txDataBuf[4] = {0x93, 0x88, 0x01, 0x6B}; // 设置零点指令
    FDCAN1_AddMessageToTxFifo(txDataBuf, (motor->CanID), FDCAN_DLC_BYTES_4);
}
/**
 * @brief 步进电机回到零点函数
 * @param motor 步进电机结构体指针
 * @note 该函数用于发送回到零点指令 电机将回到设置的零点位置 并且回零后自动将坐标零点设置为回零位置
 * @note 默认使用单圈就近回零 （即电机会选择最短路径回到零点位置） 第二位为回零模式：
 * @note - 0x00：单圈就近回零
 * @note - 0x01：单圈方向回零
 * @note - 0x02：无限位碰撞回零
 * @note - 0x03：限位回零
 * @note - 0x04：回到绝对位置坐标零点  为void StepMotor_ClearPosition(StepMotorStruct *motor)设置的坐标零点（???需要测试???）
 * @note - 0x05：回到上次掉电位置角度
 */
void StepMotor_BackToZero(StepMotorStruct *motor)
{
    uint8_t txDataBuf[4] = {0x9A, 0x00, 0x00, 0x6B}; // 回到零点指令
    FDCAN1_AddMessageToTxFifo(txDataBuf, (motor->CanID), FDCAN_DLC_BYTES_4);
}
/**
 * @brief 强制中断并退出回零操作
 * @param motor 步进电机结构体指针
 */
void StepMotor_EmergencyStop_BackToZero(StepMotorStruct *motor)
{
    uint8_t txDataBuf[3] = {0x9C, 0x48, 0x6B}; // 紧急停止指令
    FDCAN1_AddMessageToTxFifo(txDataBuf, (motor->CanID), FDCAN_DLC_BYTES_3);
}

/**
 * @brief 定时返回信息命令
 * @param motor 步进电机结构体指针
 * @param infoCode 信息代码
 * @param repeatTimes 重复发送的时间间隔 单位ms
 */
void StepMotor_SetReturnInfo_Repeatly(StepMotorStruct *motor, uint8_t infoCode, uint16_t repeatTimes)
{
    uint8_t txDataBuf[6];
    txDataBuf[0] = 0x11;                      // 功能码
    txDataBuf[1] = 0x18;                      // 辅助码
    txDataBuf[2] = infoCode;                  // 信息代码
    txDataBuf[3] = (repeatTimes >> 8) & 0xFF; // 重复发送时间间隔高字节
    txDataBuf[4] = repeatTimes & 0xFF;        // 重复发送时间间隔低字节
    txDataBuf[5] = 0x6B;                      // 校验位
    FDCAN1_AddMessageToTxFifo(txDataBuf, (motor->CanID), FDCAN_DLC_BYTES_6);
}
/**
 * @brief 步进电机读取信息
 * @param motor 步进电机结构体指针
 * @param infoCode 信息代码
 * @note 该函数用于发送读取信息指令 电机将返回指定的信息 （单次返回）
 */
void StepMotor_AskForInfo(StepMotorStruct *motor, uint8_t infoCode)
{
    uint8_t txDataBuf[2];
    txDataBuf[0] = infoCode; // 信息代码
    txDataBuf[1] = 0x6B;     // 校验位
    FDCAN1_AddMessageToTxFifo(txDataBuf, (motor->CanID), FDCAN_DLC_BYTES_2);
}

/**
 * @brief 步进电机初始化函数 不包含回零操作
 * @param motor 步进电机结构体指针
 * @param CanID 步进电机的CAN ID
 * @param dividerMode 步进电机的分频模式 (仅在Emm固件下起作用)
 * @note ### 该函数用于初始化步进电机结构体
 * @note -----
 * @note - 注意：函数执行后应该留一些时间给电机回零 这个过程中电机对外界指令不响应
 * @note #### 在初始化过程中完成了以下操作：
 * @note -----
 * @note 1. 设置步进电机默认加速度 (在设置速度模式时会使用)
 * @note 2. 设置步进电机默认运动速度 (在设置位置模式时会使用)
 * @note 3. 设置步进电机默认最大速度限制 (在设置力矩模式时会使用)
 * @note 4. 设置步进电机最大电流限制 (在限流控制时会使用)
 * @note 5. 向步进电机请求间隔发送实时位置（2s一次）（停用）
 * @note 6. 使能电机
 * @note 7. 进行回零操作(如果实测不回零的效果也很好 可以不进行回零操作)
 */
void StepMotor_Init(StepMotorStruct *motor, uint32_t motorAddr, uint16_t dividerMode)
{
    motor->CanID = motorAddr << 8;

    motor->dividerMode = dividerMode; // 设置分频模式
    motor->TurningSpeed = 600;         // 设置默认速度
    motor->TurningAccel = 600;        // 设置默认加速度
    motor->TurningMaxSpeed = 2000;     // 设置默认最大速度
    motor->TurningMaxCurrent = 2000;  // 设置默认最大电流

    // StepMotor_Reset(motor); // 重置电机状态

    StepMotor_Enable(motor); // 使能电机
    // StepMotor_SetReturnInfo_Repeatly(motor, STEPMOTOR_INFOCODE_POSITION, 2000); // 设置实时位置每2s返回一次
    // StepMotor_SetReturnInfo_Repeatly(motor, STEPMOTOR_INFOCODE_SPEED, 1500);    // 设置实时速度每1.5s返回一次 实测无法同时发送两种数据

    StepMotor_BackToZero(motor); // 回到零点位置 并且设为坐标零点
    stepMotor[StepMotorIndex] = motor;
    StepMotorIndex++;
}

#ifdef _CAN_CTRL_FRAMEVER_X
/**
 * @brief 设置步进电机转动(X) 使用默认加速度
 * @param motor 步进电机结构体指针
 * @param motorSpeed 电机速度 [-3000.0-3000.0] 单位为RPM 最小分度为0.1RPM
 * @note X固件有多种转动模式 这里选用速度模式限电流控制
 */
void StepMotor_SetTurning(StepMotorStruct *motor, double motorSpeed)
{
    if (motorSpeed < 0)
    {
        motorSpeed = -motorSpeed;
        StepMotor_SetSpeedMode_WithCurrentLimit_X(motor, STEPMOTOR_DIRECT_BACKWARD, motor->TurningAccel, (uint16_t)(motorSpeed * 10), motor->TurningMaxCurrent);
    }
    else
    {
        StepMotor_SetSpeedMode_WithCurrentLimit_X(motor, STEPMOTOR_DIRECT_FORWARD, motor->TurningAccel, (uint16_t)(motorSpeed * 10), motor->TurningMaxCurrent);
    }
}

/**
 * @brief 设置步进电机相对当前角度转动(X)使用默认速度
 * @param motor 步进电机结构体指针
 * @param angle 转动角度 [-360.0-360.0] 单位为度 最小分度为0.1度
 * @note X固件有多种位置模式 这里选用位置模式限电流控制
 */
void StepMotor_SetRelatAngle(StepMotorStruct *motor, double angle)
{
    if (angle < 0)
    {
        angle = -angle;
        StepMotor_SetPositionMode_WithCurrentLimit_X(motor, STEPMOTOR_DIRECT_BACKWARD, motor->TurningSpeed, (uint32_t)(angle * 10), StepMotor_TurningMode_RelateToRealPos, motor->TurningMaxCurrent);
    }
    else
    {
        StepMotor_SetPositionMode_WithCurrentLimit_X(motor, STEPMOTOR_DIRECT_FORWARD, motor->TurningSpeed, (uint32_t)(angle * 10), StepMotor_TurningMode_RelateToRealPos, motor->TurningMaxCurrent);
    }
}
/**
 * @brief 设置步进电机绝对角度转动(X)使用默认速度 相对于设置的坐标零点（与回零零点不同!!）
 * @param motor 步进电机结构体指针
 * @param angle 转动角度 [-360.0-360.0] 单位为度 最小分度为0.1度
 * @note X固件有多种位置模式 这里选用位置模式限电流控制
 */
void StepMotor_SetAngle(StepMotorStruct *motor, double angle)
{
    if (angle < 0)
    {
        angle = -angle;
        StepMotor_SetPositionMode_WithCurrentLimit_X(motor, STEPMOTOR_DIRECT_BACKWARD, motor->TurningSpeed, (uint32_t)(angle * 10), StepMotor_TurningMode_RelateToZero, motor->TurningMaxCurrent);
    }
    else
    {
        StepMotor_SetPositionMode_WithCurrentLimit_X(motor, STEPMOTOR_DIRECT_FORWARD, motor->TurningSpeed, (uint32_t)(angle * 10), StepMotor_TurningMode_RelateToZero, motor->TurningMaxCurrent);
    }
}
#endif //_CAN_CTRL_FRAMEVER_X

#ifdef _CAN_CTRL_FRAMEVER_Emm
/**
 * @brief 设置步进电机转动(Emm) 使用加速度为0 不设置速度曲线 直接启动
 * @param motor 步进电机结构体指针
 * @param motorSpeed 电机速度 [-3000-3000] 单位为RPM
 */
void StepMotor_SetTurning(StepMotorStruct *motor, double motorSpeed)
{
    if (motorSpeed < 0)
    {
        motorSpeed = -motorSpeed;
        StepMotor_SetSpeedMode_Emm(motor, STEPMOTOR_DIRECT_BACKWARD, 0, (uint16_t)motorSpeed);
    }
    else
    {
        StepMotor_SetSpeedMode_Emm(motor, STEPMOTOR_DIRECT_FORWARD, 0, (uint16_t)motorSpeed);
    }
}
/**
 * @brief 设置步进电机相对当前角度转动(Emm)使用默认速度
 * @param motor 步进电机结构体指针
 * @param angle 转动角度 [-360.0-360.0] 单位为度
 * @note 该函数过程中有脉冲数的计数过程 运行效率可能有点慢
 */
void StepMotor_SetRelatAngle(StepMotorStruct *motor, double angle)
{
    if (angle < 0)
    {
        angle = -angle;
        StepMotor_SetPositionMode_Emm(motor, STEPMOTOR_DIRECT_BACKWARD, motor->TurningSpeed, 0, angle, StepMotor_TurningMode_RelateToRealPos);
    }
    else
    {
        StepMotor_SetPositionMode_Emm(motor, STEPMOTOR_DIRECT_FORWARD, motor->TurningSpeed, 0, angle, StepMotor_TurningMode_RelateToRealPos);
    }
}
/**
 * @brief 设置步进电机绝对角度转动(Emm)使用默认速度 相对于设置的坐标零点（与回零零点不同!!）
 * @param motor 步进电机结构体指针
 * @param angle 转动角度 [-360.0-360.0] 单位为度
 * @note 该函数过程中有脉冲数的计算过程 运行效率可能有点慢
 */
void StepMotor_SetAngle(StepMotorStruct *motor, double angle)
{
    if (angle < 0)
    {
        angle = -angle;
        StepMotor_SetPositionMode_Emm(motor, STEPMOTOR_DIRECT_BACKWARD, motor->TurningSpeed, 0, angle, StepMotor_TurningMode_RelateToZero);
    }
    else
    {
        StepMotor_SetPositionMode_Emm(motor, STEPMOTOR_DIRECT_FORWARD, motor->TurningSpeed, 0, angle, StepMotor_TurningMode_RelateToZero);
    }
}

#endif // _CAN_CTRL_FRAMEVER_Emm

/**
 * @brief 使步进电机停止并退出当前模式 使用其他模式指令前必须调用（是否必须调用？需要测试）
 */
void StepMotor_Stop(StepMotorStruct *motor)
{
    StepMotor_EmergencyStop(motor);
}

#endif
