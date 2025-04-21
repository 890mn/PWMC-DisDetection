#ifndef __TB_TIMER_H__
#define __TB_TIMER_H__

#include "stm32f10x.h"
#include "z_main.h"

void SysTick_Int_Init(void);
void TIM1_Int_Init(u16 arr,u16 psc);
void TIM2_Int_Init(u16 arr,u16 psc);

void TIM3_Int_Init(u16 arr,u16 psc);
u32 millis(void);

void TIM3_Pwm_Init(u16 arr,u16 psc);
void TIM4_Pwm_Init(u16 arr,u16 psc);

/*******��ʱ�����ָ���*******/
#define Timeout_ON()	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE)
#define Timeout_OFF()	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE)
#define CCP_Falling()	TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Falling)
#define CCP_Rising()	TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Rising)

void IWDG_Init(void);


#endif
