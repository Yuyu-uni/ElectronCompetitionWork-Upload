#ifndef PID_H
#define PID_H
// 设置枚举类型，给pid模式选择
enum PID_Mode
{
    // 微分先行
    PID_D_First_DISABLE = 0,
    PID_D_First_ENABLE = 1,
    // 积分分离
    PID_I_Separate_DISABLE = 0,
    PID_I_Separate_ENABLE = 1,
    // 变速积分
    PID_I_Variable_Speed_DISABLE = 0,
    PID_I_Variable_Speed_ENABLE = 1,
};
//(学习笔记)结构体定义时，内部调用结构体本身，结构体名前必须加struct，且定义结构体前后都要加结构体名称。编译链是thread模式，必须遵从从前到后，必须声明了新地址，后面调用才能找到命名地址。

// 结构体定义pid参量，可以用于多环pid运算不冲突

typedef struct PIDController
{
    // public
    void (*pid_init)(struct PIDController *pid, float kp, float ki, float kd, float kf, float a, float I_Out_MAX, float Out_MAX, float D_T, float I_Variable_Speed_A, float I_Variable_Speed_B, float I_Separate_Threshold, float target, float Dead_Zone, int D_First_Mode, int I_Separate_Mode, int I_Variable_Speed_Mode);

    void (*pid_calculate)(struct PIDController *pid, float target);

    void (*pid_set_target)(struct PIDController *pid, float target);

    // protect
    float Kp; // Proportional gain
    float Ki; // Integral gain
    float Kd; // Derivative gain
    float kf; // Feedforward gain (optional, can be used for feedforward control)

    float I_Out_MAX; // Maximum output for integral term
    float Out_MAX;   // PID output MAX
    float D_T;       // Sample time in seconds
    float pre_error; // Previous error value for derivative calculation

    // 变速积分定速内段阈值, 0为不限制
    float I_Variable_Speed_A;
    // 变速积分变速区间, 0为不限制
    float I_Variable_Speed_B;
    // 积分分离阈值，需为正数, 0为不限制
    float I_Separate_Threshold;

    int D_First_Mode;          // 微分先行模式
    int I_Separate_Mode;       // 积分分离模式
    int I_Variable_Speed_Mode; // 变速积分模式

    float target;
    float pre_target;

    float Dead_Zone; // Dead zone for output
    float a;         // 低通滤波系数，为0就是不低通滤波

    float Intergral; // 积分值

} PIDController;

void pid_init(PIDController *pid, float kp, float ki, float kd, float kf, float a, float I_Out_MAX, float Out_MAX, float D_T, float I_Variable_Speed_A, float I_Variable_Speed_B, float I_Separate_Threshold, float target, float Dead_Zone, int D_First_Mode, int I_Separate_Mode, int I_Variable_Speed_Mode);

void pid_set_target(PIDController *pid, float target);

float pid_calculate(PIDController *pid, float current_value, float target);

float Angle_PIDcaculate(PIDController *pid, float angledata, float gyrodata, float target);

float PID_Calculate_DSP(PIDController *pid, float current_value, float target);

float pid_calculate_with_error(PIDController *pid, float error);

extern PIDController Angle;
extern PIDController speed;
extern PIDController turn;
extern PIDController Follow;

#endif // ! PID_H
