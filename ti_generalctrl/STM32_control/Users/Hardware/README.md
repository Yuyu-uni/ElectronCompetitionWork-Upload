# 模块说明介绍

## ADS1115 ADC检测模块

> I2C通信

- 使用详细方法

[飞书文档README](https://g167qewiw7n.feishu.cn/docx/ObzZd0nOvotGRgxzgRXcswpWnQd)

## 感为八路灰度循迹传感器

> I2C通信

目前仅实现数字量的接收，无法获取每一路的模拟量结果

- 使用详细方法

[飞书文档README](https://g167qewiw7n.feishu.cn/docx/YGSRd3TONocbDxx57ZOcuiP8n5y?from=from_copylink)

## ZX_servo-总线舵机模块

目前是边猜边写的、、、、

总之请仔细看飞书并进行测试！！

- 详细使用方法

[飞书文档README](https://g167qewiw7n.feishu.cn/docx/DD6idpBKLo6OkvxjioPccj3Wn7e?from=from_copylink)

## StepMotor-步进电机驱动

包括脉冲控制模式 占用TIM8和TIM17 & CAN通信控制模式

- 详细使用方法

[飞书文档README](https://g167qewiw7n.feishu.cn/docx/TphudtYbDoKcgxxuJctc4wT4nle)

### 更新日志

0720更新：

1. 脉冲控制模式经测试完美工作 目前细分模式仅8和128可用
2. CAN编写完毕，等待测试

0721更新：

1. 修复转动角度超出360°的bug 目前的角度范围为0-360°
2. 增加脉冲控制模式的定速旋转功能



