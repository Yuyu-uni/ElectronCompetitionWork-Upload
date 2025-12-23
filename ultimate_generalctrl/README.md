# GeneralCtrlProject_C

# 仓库介绍
电赛控制题通用工程代码仓库。该库中储存电赛控制工程C语言部分的代码

- main分支中为GeneralEmptyProject，不针对任何特定题目的普适性工程文件
- branches：每次为一特定题目创建一个branch，继承main主干中的驱动文件与总体结构性代码

# 更新记录

- 0710迁移内容：只做了简单的文件迁移、复制粘贴，未改变任何代码，并且已编译通过
- 0713：新建BSP文件，开始编写BSP驱动。
- 0718：Cube MX更新：新增USART2并重新生成工程，（理论上，对代码无任何影响）
- 0725：步进电机CAN通信方式测试完成，正常使用
  - 修改了张大头电机驱动说明书中的错误
- 0725：Cube MX更新：删除TIM1更新中断 拉高U1的rx引脚（PC5）  以防止出现跑飞的奇怪bug
  - 已重新生成代码 编译 debug均正常



# 软件架构

软件架构说明

## GeneralEmptyProject

通用控制题目工程文件

## Users
文件夹存放所有用户代码

---

### Tool

算法层代码，最底层代码

1.  PID

2.  部分数学库

3.  以后会添加其他滤波算法和控制算法之类

4.  微妙级延时~TIM6~

    ~TIM7定时采样~

---

### Hardware

包括所有外设模块的代码文件

1.  jy901
2.  FollowingLine_8chan
3.  Motor~TIM20CH1-4~
4.  Encoder~TIm5采样~  TIM1-4四个轮子
5.  ZX_Servo总线舵机和PWM舵机
6.  PWM生成*2~TIM15CH1、2~
7.  步进电机~TIM8、17脉冲计数~

---
### BSP

1.  key(4*3)种state
2.  buzzer
3.  led(on\off\laser)

~占用Tim16作为BSP库状态机~


---

###  Task

执行层代码，最上层代码，用于任务执行和状态机

---

### Application(HAl库自动生成的，用于存放stm32外设通用函数)

>   uart.c

```c
    void Uart_printf(UART_HandleTypeDef *huart, char *format, ...);

    uint8_t Command_Write(uint8_t *data, uint8_t length); // 写入数据到环形缓冲区
    uint8_t Command_GetCommand(uint8_t *command);         // 获取一条完整的指令

    extern RingBuffer RingBuffer1; // 定义环形缓冲区1
```



>   stm32g4xx_it.c

```
//用于存放各种中断，严禁将中断函数重定义到其他文件
```
------

## OtherMXs

内部包括了适用于核心板、驱动板、高级板的MX文件

注意：适用于核心板的MX文件只包含核心板上的板载外设的引脚口配置  驱动板同理

区分不同板子的MX文件的核心目的是舍弃不需要的引脚配置，例如高级板的MX文件可以适配核心板+驱动板的使用，但其中也包含了高级版需要用到的引脚口，占用资源。

建议在仅使用核心板时 使用核心板MX文件

使用核心板+驱动板时 使用驱动板MX文件

使用核心板+驱动板+高级板时 使用高级板MX文件



| **定时器** | **通道/功能**           | **用途描述**                            |
| ---------- | ----------------------- | --------------------------------------- |
| **TIM1**   | -                       | 编码器接口（Encoder）                   |
| **TIM2**   | -                       | 编码器接口（Encoder）                   |
| **TIM3**   | -                       | 编码器接口（Encoder）                   |
| **TIM4**   | -                       | 编码器接口（Encoder）                   |
| **TIM5**   | -打开后编码器测速测距   | 用于采样四个轮子的编码器信号和采集距离  |
| **TIM6**   | -无中断非阻塞式延迟     | 微秒级延时（微妙级延时）                |
| **TIM7**   | -打开后小车直接开始循迹 | 定时采样（TIM7定时采样）                |
| **TIM8**   | -非can                  | 步进电机脉冲计数（步进电机驱动）        |
| **TIM15**  | CH1, CH2                | PWM信号生成（PWM生成\*2）               |
| **TIM16**  | -打开后开始BSP库        | BSP库状态机（占用Tim16作为BSP库状态机） |
| **TIM17**  | 非can-                  | 步进电机脉冲计数（步进电机驱动）        |
| **TIM20**  | CH1, CH2, CH3, CH4      | 电机PWM输出（Motor驱动，四通道）        |