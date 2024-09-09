#include "usart.h"
#include "stm32f10x.h"

/**
 * @说明     USART2 相关GPIO和工作模式配置
 * @参数     None
 * @返回值   None
 */
void USART_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // 配置USART2 TX引脚工作模式 (PA2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置USART2 RX引脚工作模式 (PA3)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 串口2工作模式配置
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // 启用接收中断
    USART_Cmd(USART2, ENABLE);
}

/**
 * @说明     USART2字符串发送函数
 * @参数     str: 指向字符串的指针
 * @返回值   None
 */
void USART_SendString(int8_t *str)
{
    uint8_t index = 0;

    while (str[index]) {
        USART_SendData(USART2, str[index]);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) ;
        index++;
    }
}
