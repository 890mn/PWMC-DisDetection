/****************************************************************************
	*	@笔者	：	ZLTech
	*	@日期	：	2022年07月19日
	*	@所属	：	杭州众灵科技
	*	@论坛	：	www.ZL-maker.com
 ****************************************************************************/

#ifndef __IR_H__
#define __IR_H__

#include "stm32f10x.h"
#include "z_main.h"
#include "z_usart.h"
#include "z_gpio.h"

/*******红外遥控器映射表*******/
#define IR	GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)

/*******红外遥控器键码表*******/
#define IR_POWER		0xa2
#define IR_MENU			0x62
#define IR_MUTE			0xe2
#define IR_MODE			0x22
#define IR_ADD			0x02
#define IR_BACK			0xc2
#define IR_FORWARD	0xe0
#define IR_SUSPEND	0xa8
#define IR_BACKWARD 0x90
#define IR_0 				0x68
#define IR_SUBTRACT 0x98
#define IR_OK				0xb0
#define IR_1 				0x30
#define IR_2 				0x18
#define IR_3 				0x7a
#define IR_4 				0x10
#define IR_5 				0x38
#define IR_6 				0x5a
#define IR_7 				0x42
#define IR_8 				0x4a
#define IR_9 				0x52

/*******红外遥控器相关函数声明*******/
u8 ir_decode(void);									  //对4个字节的用户码和键数据码进行解码
void ir_function(void);								//红外遥控器按键功能函数

#endif
