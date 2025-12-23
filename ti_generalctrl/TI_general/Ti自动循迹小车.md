#  Ti自动循迹小车

## 1.直角转弯思路

感为识别到四个灯亮起，直角左转，   DL_TimerA_stopCounter(Pid_INST);停下pid，开环差速转过去，同时巡线循到小于三个灯，重新开始pid巡线。



## 2.圈数控制思路

```
typedef struct

{

  uint8_t rote[2]; // 控制圈数

  uint8_t state;   // 0: STOP, 1: STRAIGHT , 2直角

  float base_speed; // 基础速度用于控制直行和转弯

  float location;    // 圈数记录标志位

  float direction;  // 循迹方向数据

} FollowControl;
```

rote[0]由串口屏控制行驶圈数，rote[1]每次识别到直角++，记录圈数，为了每圈只识别一次直角，开启定时器状态机，18.9s记录一次。