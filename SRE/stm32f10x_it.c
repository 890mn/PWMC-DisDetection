#include "stm32f10x_it.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Define a binary semaphore
extern SemaphoreHandle_t xEchoSemaphore;
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

void TIM3_IRQHandler(void) {
    static uint8_t polarity = 0;

    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) {
        TIM_ICInitTypeDef TIM_ICInitStructure;

        if (polarity == 0) {
            // Rising edge capture
            rising_cnt = TIM_GetCapture1(TIM3);
            polarity = 1;

            // Switch to Falling edge capture
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
            TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
            TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
            TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
            TIM_ICInitStructure.TIM_ICFilter = 9;
            TIM_ICInit(TIM3, &TIM_ICInitStructure);
        } else {
            // Falling edge capture
            falling_cnt = TIM_GetCapture1(TIM3);
            polarity = 0;

            // Switch to Rising edge capture
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
            TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
            TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
            TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
            TIM_ICInitStructure.TIM_ICFilter = 9;
            TIM_ICInit(TIM3, &TIM_ICInitStructure);

            // Signal the task that processing is required
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(xEchoSemaphore, &xHigherPriorityTaskWoken);

            // Request a context switch if a higher priority task was woken
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    }
}
