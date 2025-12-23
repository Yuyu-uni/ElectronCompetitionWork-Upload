#include "Motor.h"
#include "ti_msp_dl_config.h"
#include "task.h"
uint8_t Follow_Data; // 用于中断，存储感为八路灰度传感器的数字数据
int8_t Follow_value; // 用于存储感为八路灰度传感器的加权输出
int Follow_num;      // 用于存储感为八路灰度传感器的通道数

// 限制电机 PWM 值在最大和最小范围内
void Limit(float *motoA, float *motoB)
{
    if (*motoA > PWM_MAX)
        *motoA = PWM_MAX;
    if (*motoA < PWM_MIN)
        *motoA = PWM_MIN;

    if (*motoB > PWM_MAX)
        *motoB = PWM_MAX;
    if (*motoB < PWM_MIN)
        *motoB = PWM_MIN;
}

// 计算绝对值
int GFP_abs(int p)
{
    return p > 0 ? p : -p;
}

// 控制电机的方向和 PWM 输出
void Load(float moto1, float moto2)
{
    // 控制电机 1 的方向
    if (moto1 > 0)
    {
        DL_GPIO_setPins(Ain_PORT, Ain_PIN_0_PIN);   // Ain1
        DL_GPIO_clearPins(Ain_PORT, Ain_PIN_1_PIN); // Ain2
    }
    else
    {
        DL_GPIO_clearPins(Ain_PORT, Ain_PIN_0_PIN); // Ain1
        DL_GPIO_setPins(Ain_PORT, Ain_PIN_1_PIN);   // Ain2
    }
    // 设置电机 1 的 PWM 值
    DL_TimerG_setCaptureCompareValue(Motor_INST, GFP_abs(moto1), GPIO_Motor_C0_IDX);

    // 控制电机 2 的方向
    if (moto2 > 0)
    {
        DL_GPIO_setPins(Bin_PORT, Bin_PIN_2_PIN);   // Bin1
        DL_GPIO_clearPins(Bin_PORT, Bin_PIN_3_PIN); // Bin2
    }
    else
    {
        DL_GPIO_clearPins(Bin_PORT, Bin_PIN_2_PIN); // Bin1Bin_PIN_3_PIN
        DL_GPIO_setPins(Bin_PORT, Bin_PIN_3_PIN);   // Bin2
    }
    // 设置电机 2 的 PWM 值
    DL_TimerG_setCaptureCompareValue(Motor_INST, GFP_abs(moto1), GPIO_Motor_C1_IDX);
}

char PWM_Zero = 0, stop = 0;

// 停止电机运行，当角度偏差超过阈值时触发
void Stop(float *Med_Jiaodu, float *Jiaodu)
{
    if (GFP_abs(*Jiaodu - *Med_Jiaodu) > 60)
    {
        Load(PWM_Zero, PWM_Zero); // 停止电机
        stop = 1;                 // 设置停止标志
    }
}

// 电机分析
uint8_t FollowingLine_Analysis(uint8_t Follow_Data)
{
    Follow_num = 0;                                          // 初始化通道数
    static int8_t weights[8] = {7, 5, 3, 1, -1, -3, -5, -7}; // 左到右权值
    int8_t output = 0;
    for (int i = 0; i < 8; i++)
    {
        if (!((Follow_Data >> i) & 0x01))
        {
            output += weights[i];
        }
        Follow_num++; // 统计通道数
    }
    if(Follow_num>=4)
    {
        follow_control.state =2;
        if(follow_control.location == 1)
        {
            follow_control.rote[1]++;
            follow_control.location = 0;//置零圈数记录位
        }
    }
    return output; // 返回加权和
}

// 传感器读取gpio三个通道，分别表示八路传感器的状态
void Follow_Sensor_Read(void)
{
    int val[3];
    Follow_Data = 0; // 清空 Follow_Data
    Follow_value = FollowingLine_Analysis(Follow_Data); // 分析传感器数据
}

void Follow_direct(void)
{
    
    int follow_sensor_data = 0;
    // 读取八路循迹传感器GPIO值并组合成8位数据
    // bit0对应sensor one, bit1对应sensor two, 以此类推
    if (DL_GPIO_readPins(Follow_one_PORT, Follow_one_PIN)) {
      follow_sensor_data |= (1 << 0); // 设置bit0
    }
    if (DL_GPIO_readPins(Follow_two_PORT, Follow_two_PIN)) {
      follow_sensor_data |= (1 << 1); // 设置bit1
    }
    if (DL_GPIO_readPins(Follow_three_PORT, Follow_three_PIN)) {
      follow_sensor_data |= (1 << 2); // 设置bit2
    }
    if (DL_GPIO_readPins(Follow_four_PORT, Follow_four_PIN)) {
      follow_sensor_data |= (1 << 3); // 设置bit3
    }
    if (DL_GPIO_readPins(Follow_five_PORT, Follow_five_PIN)) {
      follow_sensor_data |= (1 << 4); // 设置bit4
    }
    if (DL_GPIO_readPins(Follow_six_PORT, Follow_six_PIN)) {
      follow_sensor_data |= (1 << 5); // 设置bit5
    }
    if (DL_GPIO_readPins(Follow_seven_PORT, Follow_seven_PIN)) {
      follow_sensor_data |= (1 << 6); // 设置bit6
    }
    if (DL_GPIO_readPins(Follow_eight_PORT, Follow_eight_PIN)) {
      follow_sensor_data |= (1 << 7); // 设置bit7
    }
    follow_control.direction = FollowingLine_Analysis(follow_sensor_data);
}