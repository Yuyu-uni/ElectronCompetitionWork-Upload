# 25电赛模拟题

##  功能

1.  循迹功能

    ```
        HAL_I2C_Master_Receive_IT(&FOLLOWINGLINE_8CHAN_GW_I2C_Handle, FOLLOWINGLINEADDR, &Follow_Data, 1); // 八路灰度传感器中断接收数据
    ```

    ```c
    // i2c接收中断
    void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
    {
        // 八路灰度传感器
        else if (hi2c->Instance == I2C3)
        {
            HAL_I2C_Master_Receive_IT(&FOLLOWINGLINE_8CHAN_GW_I2C_Handle, FOLLOWINGLINEADDR, &Follow_Data, 1);
            follow_control.direction = FollowingLine_Analysis(Follow_Data);
        }
    }
    ```

    task.h定义了

    ```c
    FollowControl follow_control = {
        .line = 0,
        .state = 0, // 0: STOP
        .base_speed = 0.0f,
        .angle = 0.0f,
        .direction = 0.0f};
    
    ```

    用来小车循迹、转向、关停、速度

2.  点追踪功能(串口接收位置)

    ```c
    void task_point_follow(void)
    {
        // 定义增量式pid控制器
        volatile int16_t X_angle_error = (int16_t)uart_rx_value;
        volatile int16_t Y_angle_error = (int16_t)uart_rx_value;
        // 计算pid值
        float X_angle_error_output = PID_Calculate_DSP_Witherror(&point, X_angle_error); // 计算x轴角度误差
        float Y_angle_error_output = PID_Calculate_DSP_Witherror(&point, Y_angle_error); // 计算y轴角度误差
        // 调整舵机角度
        x_angle += X_angle_error_output; // 调整x轴角度
        y_angle += Y_angle_error_output; // 调整y轴角度
    
        // 限制角度范围
        x_angle = limit(x_angle, 30, 150); // 限制x轴角度在30到150度之间
        y_angle = limit(y_angle, 30, 150); // 限制y轴角度在30到150度之间
    
        // 设置舵机角度
        StepMotor_SetAngle(&stepMotorA, x_angle); // 设置步进电机A角度
        StepMotor_SetAngle(&stepMotorB, y_angle); // 设置步进电机
    }
    
    ```

     或者定时器中断里接收

    ```c
    volatile float Pointpid_output_x = PID_Calculate_DSP_Witherror(&point, x_angle); // 点位环PID计算X轴
    volatile float Pointpid_output_y = PID_Calculate_DSP_Witherror(&point, y_angle); // 点位环PID计算Y轴
    ```

    

## 思路

	1. 小车保留基本的pid和卡尔曼滤波算法功能，依赖视觉返回误差来决定点的移动方向
 	2. 小车保留多状态的循迹功能



## 外设应用参考表



| **定时器** | **通道/功能**      | **用途描述**                            |
| ---------- | ------------------ | --------------------------------------- |
| **TIM1**   | -                  | 编码器接口（Encoder）                   |
| **TIM2**   | -                  | 编码器接口（Encoder）                   |
| **TIM3**   | -                  | 编码器接口（Encoder）                   |
| **TIM4**   | -                  | 编码器接口（Encoder）                   |
| **TIM5**   | -                  | 用于采样四个轮子的编码器信号和采集距离  |
| **TIM6**   | -                  | 微秒级延时（微妙级延时）                |
| **TIM7**   | -                  | 定时采样（TIM7定时采样）                |
| **TIM8**   | -                  | 步进电机脉冲计数（步进电机驱动）        |
| **TIM15**  | CH1, CH2           | PWM信号生成（PWM生成\*2）               |
| **TIM16**  | -                  | BSP库状态机（占用Tim16作为BSP库状态机） |
| **TIM17**  | -                  | 步进电机脉冲计数（步进电机驱动）        |
| **TIM20**  | CH1, CH2, CH3, CH4 | 电机PWM输出（Motor驱动，四通道）        |