#include "led.h"
#include "stm32f10x.h"

uint16_t led[8] = {LED7,LED6,LED5,LED4,LED3,LED2,LED1,LED0};

void LED_Control(uint16_t LED,uint8_t LED_Status)
{
    if(LED_Status == 0){
        GPIO_SetBits(GPIOC,LED);
        GPIO_SetBits(GPIOD,GPIO_Pin_2);
        GPIO_ResetBits(GPIOD,GPIO_Pin_2);
    }        
    else
    {
        GPIO_ResetBits(GPIOC,LED);
        GPIO_SetBits(GPIOD,GPIO_Pin_2);
        GPIO_ResetBits(GPIOD,GPIO_Pin_2); 
    }
}

void LED_PWM(float pwm_pulse){
	uint16_t level = 0;
	uint16_t i;
	while (pwm_pulse > 0 ) {
		pwm_pulse -= 12.5;
		++level;
	}
	for (i = 0; i < level; ++i) {
		LED_Control(led[i], 1);
	}
	
}
