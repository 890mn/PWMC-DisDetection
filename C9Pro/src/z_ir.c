/****************************************************************************
	*	@笔者	：	ZLTech
	*	@日期	：	2022年07月19日
	*	@所属	：	杭州众灵科技
	*	@论坛	：	www.ZL-maker.com
	*	@功能	：	存放延时相关的函数
	*	@函数列表:
	*	1.	void ir_function(void) -- 红外遥控器按键功能函数
 ****************************************************************************/

#include "z_ir.h"

/***********************************************
	函数名称：	ir_function() 
	功能介绍：	红外遥控器按键功能函数
	函数参数：	无
	返回值：		无
 ***********************************************/
void ir_function(void)
{
	switch(ir_data[2]) {
		case IR_POWER : {
			uart1_send_str((u8 *)"IR_POWER is pressed!\r\n");
		}	break;
		case IR_MENU : {
			uart1_send_str((u8 *)"IR_MENU is pressed!\r\n");
		}	break;
		case IR_MUTE : {
			uart1_send_str((u8 *)"IR_MUTE is pressed!\r\n");
		}	break;
		case IR_ADD : {
			beep_on_times(1,100);
			ai_mode = 1;
		}	break;
		case IR_SUBTRACT : {
			beep_on_times(1,100);
			ai_mode = 2;
		}	break;
		case IR_FORWARD : {
			beep_on_times(1,100);
			ai_mode = 3;
		}	break;
		case IR_BACKWARD : {
			beep_on_times(1,100);
			ai_mode = 4;
		}	break;
		case IR_SUSPEND : {
			beep_on_times(1,100);
			ai_mode = 0;
		}	break;
		case IR_MODE : {
			beep_on_times(1,100);
			ai_mode = 5;
		}	break;
		case IR_BACK : {
			beep_on_times(1,100);
			ai_mode = 6;
		}	break;
		case IR_OK : {
			uart1_send_str((u8 *)"IR_OK is pressed!\r\n");
		}	break;
		case IR_0 : {
			uart1_send_str((u8 *)"IR_0 is pressed!\r\n");
		}	break;
		case IR_1 : {
			beep_on_times(1,100);
			ai_mode = 11;
		}	break;
		case IR_2 : {
			beep_on_times(1,100);
			ai_mode = 12;
		}	break;
		case IR_3 : {
			beep_on_times(1,100);
			ai_mode = 13;
		}	break;
		case IR_4 : {
			beep_on_times(1,100);
			ai_mode = 14;
		}	break;
		case IR_5 : {
			uart1_send_str((u8 *)"IR_5 is pressed!\r\n");
		}	break;
		case IR_6 : {
			uart1_send_str((u8 *)"IR_6 is pressed!\r\n");
		}	break;
		case IR_7 : {
			uart1_send_str((u8 *)"IR_7 is pressed!\r\n");
		}	break;
		case IR_8 : {
			uart1_send_str((u8 *)"IR_8 is pressed!\r\n");
		}	break;
		case IR_9 : {
			uart1_send_str((u8 *)"IR_9 is pressed!\r\n");
		}	break;
	}
}
