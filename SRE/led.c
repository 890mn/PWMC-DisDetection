#include "led.h"
#include "stm32f10x.h"

uint16_t led[8] = {LED7,LED6,LED5,LED4,LED3,LED2,LED1,LED0};

/**
  * @说明     控制LED打开或关闭
  * @参数     Led: LED编号,GPIO_Pin_8到GPIO_Pin_15
  * @参数     Ledstatus: 0，关闭LED；1，打开LED
  * @返回值   None
  */
void LED_Control(uint16_t LED,uint8_t LED_Status)
{
    if(LED_Status == 0){
        GPIO_SetBits(GPIOC,LED);
        GPIO_SetBits(GPIOD,GPIO_Pin_2);
        GPIO_ResetBits(GPIOD,GPIO_Pin_2);  //状态锁存
    }        
    else
    {
        GPIO_ResetBits(GPIOC,LED);
        GPIO_SetBits(GPIOD,GPIO_Pin_2);
        GPIO_ResetBits(GPIOD,GPIO_Pin_2);  //状态锁存    
    }
}

void LED_PWM(float pwm_pulse){
	uint16_t level = 0;
	uint16_t i;
	while (pwm_pulse > 0 ) {
		pwm_pulse -= 0.125;
		++level;
	}
	for (i = 0; i < level; ++i) {
		LED_Control(led[i], 1);
	}
	
}
