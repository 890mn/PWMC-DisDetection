#include "tim.h"
#include "stm32f10x.h"

/**
 * @说明     TIM3 捕获初始化 (用于超声波的 Echo 捕获)
 * @参数     None
 * @返回值   None
 */
void TIM3_Config(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // 使能 TIM3 时钟

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    // 基本定时器配置
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;      // 1MHz 计时频率
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 65535;          // 自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // 输入捕获配置，PA6 对应 TIM3_CH1
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // 上升沿捕获
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 8;  // 适当的输入滤波器值
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    // 启用TIM3
    TIM_Cmd(TIM3, ENABLE);

    // 启用输入捕获中断
    TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
}

void TIM2_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 启用 TIM2 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 基本定时器配置
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // 预分频为72，1MHz计数频率
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 910 - 1; // ARR = 1000，PWM频率为1kHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 配置 TIM2_CH1 和 TIM2_CH2 为 PWM 输出 (PA0, PA1)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // 初始占空比为 0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM2, &TIM_OCInitStructure); // 配置 TIM2_CH2 (PA1)

    // 启动 TIM2 定时器
    TIM_Cmd(TIM2, ENABLE);
}

