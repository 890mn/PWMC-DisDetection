#include <stdio.h>
#include <string.h>
#include "z_adc.h"
#include "z_global.h"
#include "z_gpio.h"
#include "z_timer.h"
#include "z_usart.h"
#include "z_main.h"
#include "z_delay.h"
#include "z_sensor.h"


/*
	智能功能代码
*/


//AI_PIN PB0  - TODO

//初始化传感器IO口-S3
void setup_sensor(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
}

void AI_jiaqu(void) {
	if(millis() < 3000)return;
	if(group_do_ok == 0)return;//有动作执行，直接返回
	//sprintf((char *)uart_receive_buf, "AI_Read() = %d\r\n", AI_Read());
	//uart1_send_str(uart_receive_buf);
	//mdelay(500);
	if(AI_Read() == 0) {
		mdelay(20);
	  if(AI_Read() == 0) {
			parse_cmd((u8 *)"$DGT:1-9,1!");
			beep_on_times(1, 100);
		} 
	}
}



