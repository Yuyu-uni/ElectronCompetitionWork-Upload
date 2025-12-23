#include "PID.h"
#include "drv_math.h"
#include "task.h"
#include "ti_msp_dl_config.h"

// 定义几个环PID控制器，同时实例化函数指针（如果不想用函数指针，可以直接调用函数，都一样，甚至可以把实例化注释了，如果不实例化直接使用函数指针，代码会跑飞，因为是野指针）
PIDController Angle;
PIDController Angle = {
    .pid_init = pid_init,
    .pid_set_target = pid_set_target, // 其它成员初始化
};

PIDController turn;
PIDController turn = {
    .pid_init = pid_init, .pid_set_target = pid_set_target,
    // 其它成员初始化
};
PIDController Follow;
PIDController Follow = {
    .pid_init = pid_init,
    .pid_set_target = pid_set_target, // 其它成员初始化
};

// // For example本代码全部函数使用案例
// pid_init(&Angle, 1, 0, 1, 1, 1000, 1000, 1, 1000, 1100, 1000, 0, 0,
// PID_D_First_DISABLE, PID_I_Separate_DISABLE, PID_I_Variable_Speed_DISABLE);
// pid_set_target(&Angle, 200);
// pid_calculate(&Angle, 100);

void pid_init(PIDController *pid, float kp, float ki, float kd, float kf,
              float a, float I_Out_MAX, float Out_MAX, float D_T,
              float I_Variable_Speed_A, float I_Variable_Speed_B,
              float I_Separate_Threshold, float target, float Dead_Zone,
              int D_First_Mode, int I_Separate_Mode,
              int I_Variable_Speed_Mode) {

  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  pid->kf = kf;

  // Initialize PID parameters
  pid->I_Out_MAX = I_Out_MAX;
  pid->Out_MAX = Out_MAX;
  pid->D_T = D_T;
  pid->I_Variable_Speed_A = I_Variable_Speed_A;
  pid->I_Variable_Speed_B = I_Variable_Speed_B;
  pid->I_Separate_Threshold = I_Separate_Threshold;

  pid->a = a; // Initialize low-pass filter coefficient

  pid->target = target;   // Initialize target to 0
  pid->pre_target = 0.0f; // Initialize pre_target to 0
  pid->pre_error = 0.0f;  // Initialize previous error to 0
  pid->Dead_Zone = Dead_Zone;
  pid->Intergral = 0.0f; // Initialize integral term to 0

  // Initialize modes
  pid->D_First_Mode = D_First_Mode;
  pid->I_Separate_Mode = I_Separate_Mode;
  pid->I_Variable_Speed_Mode = I_Variable_Speed_Mode;
}

void pid_set_target(PIDController *pid, float target) {
  pid->pre_target = pid->target; // Store the previous target value
  pid->target = target;
}

float pid_calculate(PIDController *pid, float current_value, float target) {
  float speed_ratio;
  // Calculate error
  float error = target - current_value;
  // 低通滤波,a为0时不进行低通滤波
  error = (1 - pid->a) * error + pid->a * pid->pre_error;
  float abs_error = myabs(error);
  // 判断死区
  if (abs_error < pid->Dead_Zone) {
    error = 0.0f;     // If within dead zone, set error to 0
    abs_error = 0.0f; // Reset absolute error
    // pid->target = current_value; // Update target to current value
  }
  float P_Out = pid->Kp * error;
  float I_Out = pid->Ki * pid->Intergral;
  pid->Intergral += error * speed_ratio * pid->D_T;
  // 积分限幅
  pid->Intergral = limit(pid->Intergral, -pid->I_Out_MAX / pid->Ki,
                         pid->I_Out_MAX / pid->Ki);
  I_Out = pid->Ki * pid->Intergral;
  float D_Out = pid->Kd * (error - pid->pre_error) /
          pid->D_T; // Derivative of the current value
  // Feedforward term
  float F_Out = pid->kf * (target - pid->pre_target);
  // Calculate total output
  float output = P_Out + I_Out + D_Out + F_Out;
  // Clamp output to maximum limits
  if (output > pid->Out_MAX)
    output = pid->Out_MAX;
  else if (output < -pid->Out_MAX)
    output = -pid->Out_MAX;

  // 善后工作
  pid->pre_error = error;   // Update previous error for next iteration
  pid->pre_target = target; // Update previous target for next iteration

  return output;
}

float pid_calculate_with_error(PIDController *pid, float error) {
  float speed_ratio;

  // 低通滤波,a为0时不进行低通滤波
  error = (1 - pid->a) * error + pid->a * pid->pre_error;

  float abs_error = myabs(error);

  // 判断死区
  if (abs_error < pid->Dead_Zone) {
    error = 0.0f;     // If within dead zone, set error to 0
    abs_error = 0.0f; // Reset absolute error
  }

  // Proportional term
  float P_Out = pid->Kp * error;

  /************************************************************************* */

  // Integral term

  // 如果是非变速积分，增长速率为1，否则则为一定斜率增长
  if (pid->I_Variable_Speed_Mode == PID_I_Variable_Speed_DISABLE) {
    speed_ratio = 1;
  } else {
    // 变速积分
    if (abs_error <= pid->I_Variable_Speed_B) {
      speed_ratio = 1.0f;
    } else if (pid->I_Variable_Speed_B < abs_error &&
               abs_error < pid->I_Variable_Speed_A + pid->I_Variable_Speed_B) {
      speed_ratio =
          (pid->I_Variable_Speed_A + pid->I_Variable_Speed_B - abs_error) /
          pid->I_Variable_Speed_A;
    }
    if (abs_error >= pid->I_Variable_Speed_B) {
      speed_ratio = 0.0f;
    }
  }

  // 积分限幅
  pid->Intergral = limit(pid->Intergral, -pid->I_Out_MAX / pid->Ki,
                         pid->I_Out_MAX / pid->Ki);
  float I_Out = pid->Ki * pid->Intergral;

  // 积分分离
  if (pid->I_Separate_Mode == PID_I_Separate_ENABLE &&
      abs_error > pid->I_Separate_Threshold) {
    pid->Intergral = 0.0f; // Reset integral if error exceeds threshold
    I_Out = 0.0f;          // Set integral output to 0
  } else {
    // Update integral term with clamping
    pid->Intergral += error * speed_ratio * pid->D_T;
    I_Out = pid->Ki * pid->Intergral;
  }

  /************************************************************************* */
  // Derivative term
  float D_Out = 0.0f;
  if (pid->D_First_Mode == PID_D_First_ENABLE) {
    D_Out = pid->Kd * (pid->target - pid->pre_target) /
            pid->D_T; // Derivative of the target
  } else {
    D_Out = pid->Kd * (error - pid->pre_error) /
            pid->D_T; // Derivative of the error
  }

  // Feedforward term
  float F_Out = pid->kf * (pid->target - pid->pre_target);

  // Calculate total output
  float output = P_Out + I_Out + D_Out + F_Out;

  // Clamp output to maximum limits
  if (output > pid->Out_MAX)
    output = pid->Out_MAX;
  else if (output < -pid->Out_MAX)
    output = -pid->Out_MAX;

  // 善后工作
  pid->pre_error = error; // Update previous error for next iteration

  return output;
}

float Angle_PIDcaculate(PIDController *pid, float angledata, float gyrodata,
                        float target) {
  // 计算角度误差，限制到-180到180度
  float error = angle_wrap(target - angledata);

  // 积分项累加
  pid->Intergral += error * pid->D_T;

  // 积分限幅
  if (pid->Intergral > pid->I_Out_MAX / pid->Ki)
    pid->Intergral = pid->I_Out_MAX / pid->Ki;
  else if (pid->Intergral < -pid->I_Out_MAX / pid->Ki)
    pid->Intergral = -pid->I_Out_MAX / pid->Ki;

  // 计算PID输出
  float output =
      pid->Kp * error + pid->Ki * pid->Intergral + pid->Kd * gyrodata;

  // 输出限幅
  if (output > pid->Out_MAX)
    output = pid->Out_MAX;
  else if (output < -pid->Out_MAX)
    output = -pid->Out_MAX;

  return output;
}
