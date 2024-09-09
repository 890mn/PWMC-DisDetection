#include "gpio.h"
#include "stm32f10x.h"

/**
 * @说明     GPIO配置函数 (PA6 - Echo, PA7 - Trigger)
 * @参数     None
 * @返回值   None
 */
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能 GPIOA 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    //LED引脚配置，PC08~PC15
    GPIO_InitStructure.GPIO_Pin = LED0 |LED1 | LED2 | LED3 | LED4 | LED5 | LED6\
                                  | LED7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //74HC573锁存引脚配置，PD2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
    /* 配置 PA6 为输入浮空模式 (Echo) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 配置 PA7 为推挽输出模式 (Trigger) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置 PA0 和 PA1 为 PWM 输出 (TIM2_CH1, TIM2_CH2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 设置 PA7 输出低电平 */
    GPIO_ResetBits(GPIOA, GPIO_Pin_7);
    LED_Control(LEDALL, 0);
}
