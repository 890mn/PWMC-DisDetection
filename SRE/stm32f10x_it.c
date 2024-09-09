#include "stm32f10x_it.h"

extern uint32_t TimingDelay;

extern uint16_t rising_cnt;
extern uint16_t falling_cnt;
extern uint8_t echo_flag;

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	TimingDelay--;
}

/**
 * @说明     TIM3捕获中断处理函数
 * @参数     None
 * @返回值   None
 */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
    {
        if (!echo_flag)
        {
            // 捕获到上升沿
            rising_cnt = TIM_GetCapture1(TIM3);
            echo_flag = 1;

            // 切换到下降沿捕获
            TIM_ICInitTypeDef TIM_ICInitStructure;
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
            TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; // 设置为下降沿捕获
            TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
            TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
            TIM_ICInitStructure.TIM_ICFilter = 8;
            TIM_ICInit(TIM3, &TIM_ICInitStructure);
        }
        else
        {
            // 捕获到下降沿
            falling_cnt = TIM_GetCapture1(TIM3);
            echo_flag = 0;

            // 恢复为上升沿捕获
            TIM_ICInitTypeDef TIM_ICInitStructure;
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
            TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // 设置为上升沿捕获
            TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
            TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
            TIM_ICInitStructure.TIM_ICFilter = 8;
            TIM_ICInit(TIM3, &TIM_ICInitStructure);
        }

        // 清除中断标志位
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    }
}
