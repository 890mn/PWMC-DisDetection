# Ultrasonic Distance-Based Motor Controller System

- CT117E 嵌入式竞赛板 搭载 STM32F103RBT6
- 超声波测距模块 HC-SR04
- 电机驱动模块 DRV8833
- 蓝牙串口模块 DX-BT24
- 代码遵循 Doxygen 注释规范，精简目录结构

---

## 项目研究方向

### 1. 信息在不同观测角度的抽象能力提升

即使不用精确的数值也能让整体的状态一目了然，
无论从何种角度都能有一定的信息获取，以设计易于理解的展示方式为目标

### 2. 直流电机PWM低占空比降噪及存储结构优化

针对低占空比电机空耗问题进行软件消抖，
在测距和占空比之间设计测试一套拟合曲线用于最大程度的电机输出优化，
对返回的数值进行精度保留并采用缩放因子进行整型保存

### 3. LCD竖向显示及局部滤波更新状态系统

设计字库转换、绘制图形等函数适配竖向LCD显示，
刷新方式采用滤波局部更新减少闪烁使得总体框架更加流畅丝滑

---

## 项目部分展示

### CT117E开发板与模块连接图

|  Module  |     Pin-1     |   Pin-2    | GND | Vcc |
| :------: | ------------: | ---------: | :-: | :-: |
| HC-SR04  | Trigger - PA7 | Echo - PA6 |  -  |  5  |
| DRV8833  | IN1     - PA1 | IN2  - NaN |  -  |  5  |
| DX-BT24  | TX      - PA3 | RX   - PA2 |  -  |  5  |

### 演示视频

![旧版组合图](https://images.weserv.nl/?url=https://101.132.75.60/wp-content/uploads/2024/09/Physical-picture.jpg "组合图")

[超声波测距演示](https://link2hinar.fun/wp-content/uploads/2024/09/Ultrasonic.mp4) | [蓝牙串口演示](https://link2hinar.fun/wp-content/uploads/2024/09/Bluetooth.mp4)

---

## 项目细节

### 信息在不同观测角度的抽象能力提升

如果一个设备上有这样三个输出，他们分别代表了一种数据展示的常见方式：

1. 蓝牙串口输出精确数值
2. LCD屏幕显示精确/图形曲线表示
3. LED流水灯抽象数据形式

那么，做出各自的特色不就能够实现 1 + 1 + 1 > 3 了吗

1. 蓝牙串口擅长数据高速传输-------------精确数值管理
2. LCD屏幕大而直观-----------------------绘制图形动态表示
3. LED只有二象性状态---------------------类电量显示抽象数据

前两个设计在视频中有所涉及，而LED显示则是新引入的特性，从直接抽象占空比显示改为表示0-65%的占空比状态，由LCD表示66-100%的占空比状态

这一特性基于电机在65%的占空比下能够进入一定效率的工作的前提下，当超声波传感器检测到过远的信号传入时占空比即会徘徊在较低的水平，一方面展示了在超过工作范围时各部件的工作状态；

另一方面通过对65分为八部分可以粗略估计占空比数值进而对距离有一定的模糊感知，相比蓝牙串口给了一种新的方式观测状态，即无论在何种观测角度都保留了一定的信息展示能力

这便是对于这个项目，我对信息抽象程度控制的理解，接下来的部分，你将会看到具体实现的一些思路、代码编写以及我的另一些想法

---

### LCD 竖置显示转换思路及实现

屏幕本身为一块2.4寸的TFT(Thin-Film Transistor) LCD，驱动采用寄存器 R0 - R239 配置LCD的工作模式、绘制图像和字符，以及控制其他参数，通过调用LCD_WriteReg函数即可以对这些寄存器进行写入操作，以实现对屏幕的控制

    #define R0 0x00
    #define R1 0x01
    #define R2 0x02
    #define R3 0x03
    ...
    #define R227 0xE3
    #define R229 0xE5
    #define R231 0xE7
    #define R239 0xEF

将屏幕竖置，此时问题来了，怎么修改驱动可以适配？

1. 直接修改寄存器配置实现硬件翻转，但不同LCD之间兼容性较差
2. 采用软件模拟实现渲染翻转，但性能消耗大，需要进一步优化

显而易见，第二种方式更有研究性，且能够为未来其他研究作为底层方案作为适配支持

---

#### 基于矩阵转置的TFT LCD竖置显示驱动修改

在本项目中，所用到的竖置显示类型分为两种，一是对于图形、线条的绘制

另一种则是对于ASCII码英文字符的竖向排列

图形的绘制相对简单，可以保持原来的渲染方向通过先旋转后生成字模或是点线面的结合

字母的旋转则是有些挑战性，我在模拟像素点绘制多个字母在横向以及竖向的对比中考虑了一种基于线性代数的字模矩阵操作方案

字模在旋转前后呈现了一种中心对称的性质，这让我想起了矩阵转置的概念，即原始字模经过转置即可得到旋转后的字模，这便是第一版的思路

但在转置的实践中发现，对于16X24的字模，盲目转置会导致显示缺少底部部分细节，进而我将转置后的字模拆分为两个12X16组成的高低位字模库，在渲染中能够最大程度的保留细节：

![ ](https://images.weserv.nl/?url=https://101.132.75.60/wp-content/uploads/2024/09/martix.jpg "Martix转置流程草图")

接下来，结合代码来详细说明：

    /**
    * @brief Display a character in vertical mode on the LCD.
    * 
    * @param Xpos The starting X coordinate for the character.
    * @param Ypos The starting Y coordinate for the character.
    * @param input The input character matrix (24x16).
    */
    void LCD_VerticalDisplay(u8 Xpos, u16 Ypos, uint16_t input[CHAR_HEIGHT]);

由于竖向打印较横向宽度短，自定义程度高，故顶层只封装到按字符打印而不是字符串

而顶层函数通过调用下面两个部分的函数实现

    /**
    * @brief Transpose a character matrix for vertical display.
    * 
    * This function takes a horizontally-aligned character matrix (input) and transposes it 
    * for vertical display. 
    * 
    * Each column is split into two 12-bit halves for drawing.
    * 
    * @param input The input character matrix (24x16).
    * @param output The transposed output matrix (each element contains 12 bits for high and low parts).
    */
    void transposeMatrix(uint16_t input[CHAR_HEIGHT], uint16_t output[CHAR_HEIGHT / 2][2]);
第一部分由矩阵转置模块实现，对字模进行转换后交给下一个函数进行绘制

转置区别高位与低位，按位遍历转置时间复杂度O(CHAR_HEIGHT x CHAR_WIDTH)，不太能进一步降低时间复杂度至O(log(CHAR_HEIGHT) x log(CHAR_WIDTH))

如遇性能瓶颈或可尝试空间换时间，将转换完成的字模库预存储减少性能消耗，但扩展为大型字模库后占用过大，如果同时兼容横向和竖向对单片机FLASH要求高，兼容性下降，故保持为原始矩阵转换不变

    /**
    * @brief Draw a character on the LCD screen.
    * 
    * output[i][0] 
    * -> low pixel in i's column  
    * 
    * output[i][1] 
    * -> high pixel in i's column 
    * 
    * Draw from low 12-bit to high 12-bit
    * 
    * @param Xpos The starting X coordinate for the character.
    * @param Ypos The starting Y coordinate for the character.
    * @param c A pointer to the character data (16 columns, 12 + 12 bits per column).
    */
    void PRO_DrawChar(u8 Xpos, u16 Ypos, uc16 *c);
第二部分由绘制函数实现，通过对高低位区分实现细节保留

---

#### 引入滤波优化的部分刷新机制

字符的竖向处理固然有些棘手，接踵而至的显示刷新则是又提出了新的问题，如何保证内容完整的同时提高屏幕刷新率，看起来更加流畅？

1. 将I2C驱动改为SPI，传输速率自然上升
2. 配置DMA加速传输，缓解MCU压力
3. 将全部刷新改为部分刷新
4. 引入缓存机制

不难发现优化的机制非常的多，但出于对这个项目的多次考量，我最终选择的组合方案如标题所示：**引入滤波优化的部分刷新机制**

项目中采用的多是静态显示，需要动态刷新的是代表距离及占空比的两个矩形条，显然在当前状态下，局部刷新足够应对显示问题

为了进一步的优化，我再加入了滤波机制确保矩形条的丝滑变换，以确保得到最佳的显示

接下来，结合代码来详细说明：

    /**
    * @brief Update the display of two octagons and their corresponding bar graphs based on distance and PWM pulse.
    * 
    * This function updates two vertical bar graphs, each associated with an octagon, based on the provided distance 
    * and PWM pulse values. The bar graphs are drawn in blocks, and their height is updated incrementally to reflect 
    * the new values. The bars are drawn or cleared incrementally based on the change in values.
    * 
    * The left-side bar represents the filtered distance, and the right-side bar represents the PWM duty cycle.
    * Octagons are drawn once to surround the bars.
    * 
    * @param distance The current distance value to be represented on the left bar graph.
    * @param pwm_pulse The current PWM pulse value to be represented on the right bar graph.
    */
    void Update_Octagons(float distance, float pwm_pulse);

主更新函数包括了框架及更新逻辑

初始化静态显示坐标，进而计算滤波加快矩形条更新进度

滤波原始作用是减少信号中的噪声和干扰，从而得到更加稳定和准确的测量值
作用于部分刷新模块时得到了减少抖动以及平滑变化的效果，使得整体的观感上升不少

通过此处滤波自适应的机制，当新数据与滤波后的平均值相差较大时，会直接返回新数据，从而提高系统在变化较大时的响应速度，避免滤波导致系统响应滞后

滤波后进一步计算是否更新，通过快速响应机制改变矩形条的显示

    /**
    * @brief Filling a matrix bit-by-bit
    * 
    * @param x The X-coordinate of the start point.
    * @param y The Y-coordinate of the start point.
    * @param width The width of the martix.
    * @param height The height of the martix.
    * @param color The color of the martix.
    */
    void LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);

通过简单的 O(HEIGHT x WIDTH) 时间复杂度进行按点填充即可完成更新

---

### 直流电机PWM低占空比优化及存储结构优化

初步使得项目跑起来之后，细化功能实现以及对存储、算法的进一步优化则是这个阶段最为重要的一环，对于这个项目来说也就是如下这两个方面：

1. 在低占空比运行状态下的直流电机噪音及转换效率
2. 多处采用float进行精度控制可能造成的性能降低

对于精度问题，经过实测数据对比分析后，我最终保留了一部分刷新机制及电机控制的float，而在一些例如查表、缓存的低需求部分采用了缩放因子的等比转换，保证精度的同时提高了处理速度

    #define STEP_01_10 1     // STEP = 0.1 -> 1
    #define MAX_X_01_10 200  // MAX_X = 20 -> 200

而在低占空比的处理上，我又进一步设计了一套拟合函数贴合使用体验，通过数学处理表现出了非线性的细腻调速：

![ ](https://images.weserv.nl/?url=https://101.132.75.60/wp-content/uploads/2024/09/pwm.jpg "拟合曲线草图")

由于建系时进行了一些线性缩小，曲线的走势会有些出入，但如最初的设想一致即：

1. 高占空比时走势较陡，但延缓了中低占空比的比例，使得总体电机转速恒定在柔和档位附近
2. 同时在PWM设置上提高了工作频率，减少了低频工作噪声

---

## 项目总结

构建本项目前其实没有想那么多，利用手头的零散模块进行了一些小测试后逐渐有了想优化的点和想做的功能，在过程中也是对接线、配置等平时不起眼的地方有了更多的认识，对可能出现的问题也有方向去解决，给我带来的收获很多

后续再进行项目构建时还需要对数据结构的设计多留心，对于精度的控制还需要进行提前的构思、宏定义以便全局的更改以及优化
