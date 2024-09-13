# Ultrasonic Distance-Based Motor Controller System

- CT117E 嵌入式竞赛板 搭载 STM32F103RBT6
- 超声波测距模块 HC-SR04
- 电机驱动模块 DRV8833
- 蓝牙串口模块 DX-BT24
- 代码遵循 Doxygen 注释规范，精简目录结构

## 项目研究方向

1. **信息在不同观测角度的抽象能力提升**

    即使不用精确的数值也能让整体的状态一目了然，无论从何种角度都能有一定的信息获取，以设计易于理解的展示方式为目标

2. **直流电机PWM低占空比降噪及存储结构优化**

    针对低占空比电机空耗问题进行软件消抖，在测距和占空比之间设计测试一套拟合曲线用于最大程度的电机输出优化，对返回的数值进行精度保留并采用缩放因子进行整型保存

3. **LCD竖向显示及局部滤波更新状态系统**

    设计字库转换、绘制图形等函数适配竖向LCD显示，刷新方式采用滤波局部更新减少闪烁使得总体框架更加流畅丝滑

---

## 项目部分展示

### CT117E开发板与模块连接图

|  Module  |     Pin-1     |   Pin-2    | GND | Vcc |
| :------: | ------------: | ---------: | :-: | :-: |
| HC-SR04  | Trigger - PA7 | Echo - PA6 |  -  |  5  |
| DRV8833  | IN1     - PA1 | IN2  - NaN |  -  |  5  |
| DX-BT24  | TX      - PA3 | RX   - PA2 |  -  |  5  |

### 演示视频
[超声波测距演示](https://github.com/890mn/PWMC-DisDetection/blob/main/Video-Pic/Ultrasonic.mp4)
[蓝牙串口演示](https://github.com/890mn/PWMC-DisDetection/blob/main/Video-Pic/Bluetooth.mp4)
