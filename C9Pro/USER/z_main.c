/***************************************************************
	*	@笔者	：	tacbo
	*	@日期	：	2020年08月23日
	*	@所属	：	杭州众灵科技
	*	@论坛	：	www.ZL-robot.com
	*	@功能	：	ZL-KPZ32控制板

	智能小车C3出厂程序
	实现的功能：
	1、功能：
				前进、后退、左转、右转、左平移、右平移、停止
				定距跟随、自由避障、智能循迹、循迹避障
	2、控制方式：
				红外遥控器、手柄、微信小程序、语音控制、wifi摄像头模块手机app控制
	
	传感器引脚:
		循迹（S1-PA0 PA1） 
		超声波(S3-PB0 PA2) 
	蜂鸣器引脚：
		BEEP-PB5
	LED引脚：
		NLED-PB13
  PS2手柄引脚：	
	  PS1-DAT-PA15
	  PS2-CMD-PA14
	  PS6-ATT-PA13
	  PS7-CLK-PA12
	按键引脚：
	  KEY1-PA8 KEY2-PA11
	
	统一总线口： TX3 RX3
	
	主频：72M
	单片机型号：STM32F103C8T6
	
***************************************************************/

#include "z_rcc.h"		//配置时钟文件
#include "z_gpio.h"		//配置IO口文件
#include "z_global.h"	//存放全局变量
#include "z_delay.h"	//存放延时函数
#include "z_type.h"		//存放类型定义
#include "z_usart.h"	//存放串口功能文件
#include "z_timer.h"	//存放定时器功能文件
#include "z_ps2.h"		//存放索尼手柄
#include "z_w25q64.h"	//存储芯片的操作
#include "z_adc.h"		//ADC初始化
#include <stdio.h>		//标准库文件
#include <string.h>		//标准库文件
#include <math.h>	  	//标准库文件
#include "z_kinematics.h"	//逆运动学算法
#include "z_action.h" //动作组执行文件
#include "stm32f10x_iwdg.h"
#include "z_sensor.h"

#define MODULE "Jibot1-32"

/*
	全局变量定义
*/
u8 i = 0;
u8 ir_data[4],ir_flag;
u8 psx_buf[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //存储手柄的数据 	
u8 voice_flag = 0;      //用于控制语音识别时小车的基本动作执行时间为2秒
int xunji_speed = 300;  //循迹速度,范围（0-1000），可根据实际情况修改循迹速度
u8 ai_mode = 0;
/*
ai_mode=0 时，停止
ai_mode=1 时，前进
ai_mode=2 时，后退
ai_mode=3 时，左转
ai_mode=4 时，右转
ai_mode=5 时，左平移
ai_mode=6 时，右平移
ai_mode=11时，定距跟随
ai_mode=12时，自由避障
ai_mode=13时，智能循迹

*/

kinematics_t kinematics;
/*
const char *pre_cmd_set_red[PSX_BUTTON_NUM] = {//红灯模式下按键的配置			
	"<PS2_RED01:#005P0600T2000!^#005PDST!>",	//L2						  
	"<PS2_RED02:#005P2400T2000!^#005PDST!>",	//R2						  
	"<PS2_RED03:#004P0600T2000!^#004PDST!>",	//L1						  
	"<PS2_RED04:#004P2400T2000!^#004PDST!>",	//R1			
	"<PS2_RED05:#002P2400T2000!^#002PDST!>",	//RU						  
	"<PS2_RED06:#003P2400T2000!^#003PDST!>",	//RR						  
	"<PS2_RED07:#002P0600T2000!^#002PDST!>",	//RD						  
	"<PS2_RED08:#003P0600T2000!^#003PDST!>",	//RL				
	"<PS2_RED09:$MODE!>",					            //SE    				  
	"<PS2_RED10:>",					                  //AL						   
	"<PS2_RED11:>",					                  //AR						  
	"<PS2_RED12:$DJR!>",					            //ST		
	"<PS2_RED13:#001P0600T2000!^#001PDST!>",	//LU						  
	"<PS2_RED14:#000P0600T2000!^#000PDST!>",	//LR								  
	"<PS2_RED15:#001P2400T2000!^#001PDST!>",	//LD						  
	"<PS2_RED16:#000P2400T2000!^#000PDST!>",	//LL								
};
*/

//绿灯模式下，摇杆会映射到按键，防止影响，去掉绿灯模式功能
/*
const char *pre_cmd_set_grn[PSX_BUTTON_NUM] = {//绿灯模式下按键的配置			
	"<PS2_GRN01:#005P0600T2000!^#005PDST!>",	 //L2						  
	"<PS2_GRN02:#005P2400T2000!^#005PDST!>",	 //R2						  
	"<PS2_GRN03:#004P0600T2000!^#004PDST!>",	 //L1						  
	"<PS2_GRN04:#004P2400T2000!^#004PDST!>",	 //R1			
	"<PS2_GRN05:#002P2400T2000!^#002PDST!>",	 //RU						  
	"<PS2_GRN06:#003P2400T2000!^#003PDST!>",	 //RR						  
	"<PS2_GRN07:#002P0600T2000!^#002PDST!>",	 //RD						  
	"<PS2_GRN08:#003P0600T2000!^#003PDST!>",	 //RL				
	"<PS2_GRN09:$DJR!>",					             //SE    				  
	"<PS2_GRN10:>",					                   //AL						   
	"<PS2_GRN11:>",					                   //AR						  
	"<PS2_GRN12:$DJR!>",					             //ST		
	"<PS2_GRN13:#001P0600T2000!^#001PDST!>",	 //LU						  
	"<PS2_GRN14:#000P0600T2000!^#000PDST!>",	 //LR								  
	"<PS2_GRN15:#001P2400T2000!^#001PDST!>",	 //LD						  
	"<PS2_GRN16:#000P2400T2000!^#000PDST!>",	 //LL								
};
*/

/*-------------------------------------------------------------------------------------------------------
*  程序从这里执行				
*  这个启动代码 完成时钟配置 使用外部晶振作为STM32的运行时钟 并倍频到72M
-------------------------------------------------------------------------------------------------------*/

int main(void) {	
	setup_rcc();		  //初始化时钟
	setup_global();		//初始化全局变量
	setup_gpio();		  //初始化IO口
	setup_nled();		  //初始化工作指示灯
	setup_beep();		  //初始化定时器
	setup_djio();		  //初始化舵机IO口
	setup_w25q64();		//初始化存储器W25Q64
	setup_ps2();		  //初始化PS2手柄
	setup_ir();				//初始化红外遥控器，红外接收头接A8引脚
	setup_uart1();		//初始化串口1 用于下载动作组
	setup_uart3();		//初始化串口3 用于底板总线、蓝牙、lora
	setup_systick();	//初始化滴答时钟，1S增加一次millis()的值
	setup_dj_timer();	//初始化定时器2 处理舵机PWM输出	
	setup_interrupt();//初始化总中断		
	setup_kinematics(110, 105, 75, 190, &kinematics); //kinematics 90mm 105mm 98mm 150mm
	setup_servo_bias();  //初始化舵机，将偏差代入初始值
	IWDG_Init();       //初始化独立看门狗
	setup_start();		//初始化启动信号
	setup_do_group(); //开机动作
	setup_sensor();
	
	while(1) {
		loop_nled();	  	//循环执行工作指示灯，500ms跳动一次
		loop_uart();		  //串口数据接收处理
		//loop_action();	  //动作组批量执行
		loop_ps2_data();  //循环读取PS2手柄数据
		loop_ps2_button();//处理手柄上的按钮
		loop_ps2_car();   //处理手柄摇杆数据，控制电机转动
		loop_ir();	      //循环读取红外遥控器数据
		//loop_monitor();  //定时保存一些变量
		loop_AI();		    //执行对应功能
	}
}

//--------------------------------------------------------------------------------
/*
	初始化函数实现
*/

void setup_rcc(void) {   //初始化时钟
	tb_rcc_init();	  	   //时钟初始化
}

void setup_global(void) {//初始化全局变量
	tb_global_init();	
}

void setup_gpio(void) {  //初始化IO口
	tb_gpio_init();		    
}

void setup_nled(void) {  //初始化工作指示灯
	nled_init();		
	nled_off();		         //工作指示灯关闭
}

void setup_beep(void) {  //初始化定时器蜂鸣器
	beep_init();		
	beep_off();			       //关闭蜂鸣器
}			
void setup_w25q64(void) {//初始化存储器W25Q64
	u8 i;
	spiFlahsOn(1);
	w25x_init();				   //动作组存储芯片初始化
	w25x_read((u8 *)(&eeprom_info), W25Q64_INFO_ADDR_SAVE_STR, sizeof(eeprom_info));//读取全局变量
	
	if(eeprom_info.version != VERSION) {//判断版本是否是当前版本
		eeprom_info.version = VERSION;		//复制当前版本
		eeprom_info.dj_record_num = 0;		//学习动作组变量赋值0
	}
	
	if(eeprom_info.dj_bias_pwm[DJ_NUM] != FLAG_VERIFY) {
		for(i=0;i<DJ_NUM;i++) {
			eeprom_info.dj_bias_pwm[i] = 0;
		}
		eeprom_info.dj_bias_pwm[DJ_NUM] = FLAG_VERIFY;
	}
	
	for(i=0;i<DJ_NUM;i++) {
		duoji_doing[i].aim = 1500 + eeprom_info.dj_bias_pwm[i];
		duoji_doing[i].cur = 1500 + eeprom_info.dj_bias_pwm[i];
		duoji_doing[i].inc = 0;
	}
	spiFlahsOn(0);
}	

void setup_adc(void) {//初始化ADC采集 使用DMA初始化
	ADC_init();
}

void setup_ps2(void) {//初始化PS2手柄
	PSX_init();	
}

void setup_ir(void) { //初始化红外遥控器
	timer1_ir_init(65535,71);
}

void setup_uart1(void) {
  //串口1初始化
	tb_usart1_init(115200);
	//串口1打开
	uart1_open();
	//串口发送测试字符
	uart1_send_str((u8 *)"uart1 check ok!");
}
//初始化串口2
void setup_uart2(void) {
	//串口2初始化
	tb_usart2_init(115200);
	//串口2打开
	uart2_open();
	//串口发送测试字符
	uart2_send_str((u8 *)"uart2 check ok!");
}	
//初始化串口3
void setup_uart3(void) {
	//串口3初始化
	tb_usart3_init(115200);
	//串口3打开
	uart3_open();
	//串口发送测试字符
	uart3_send_str((u8 *)"uart3 check ok!");
	//总线输出 复位总线舵机 串口3即为总线串口
	zx_uart_send_str((u8 *)"#255P1500T2000!");
}	
//初始化滴答时钟，1S增加一次millis()的值
void setup_systick(void) {
	//系统滴答时钟初始化	
	SysTick_Int_Init();
}	
//初始化启动信号
void setup_start(void) {
	//蜂鸣器LED 名叫闪烁 示意系统启动
	beep_on();nled_on();tb_delay_ms(100);beep_off();nled_off();tb_delay_ms(100);
	beep_on();nled_on();tb_delay_ms(100);beep_off();nled_off();tb_delay_ms(100);
	beep_on();nled_on();tb_delay_ms(100);beep_off();nled_off();tb_delay_ms(100);
}	


//初始化总中断
void setup_interrupt(void) {
	//总中断打开
	tb_interrupt_open();
}	
//--------------------------------------------------------------------------------


//--------------------------------------------------------------------------------
/*
	主循环函数实现
*/
//循环执行工作指示灯，500ms跳动一次
void loop_nled(void) {
	static u32 time_count=0;
	static u8 flag = 0;
	if(millis()-time_count > 1000)  {
		time_count = millis();
		if(flag) {
			nled_on();
		} else {
			nled_off();
		}
		flag= ~flag;
	}
}		
//串口数据接收处理
void loop_uart(void) {
	if(uart1_get_ok) {
		if(uart1_mode == 1) {					    //命令模式
			parse_group_cmd(uart_receive_buf);
			beep_on_times(1,100);
			parse_cmd(uart_receive_buf);			
		} else if(uart1_mode == 2) {			//单个舵机调试
			parse_action(uart_receive_buf);
		} else if(uart1_mode == 3) {		  //多路舵机调试
			parse_action(uart_receive_buf);
		} else if(uart1_mode == 4) {		  //存储模式
			save_action(uart_receive_buf);
		} 
		uart1_mode = 0;
		uart1_get_ok = 0;
		uart1_open();
	}
	return;
}	

//循环读取PS2手柄数据
void loop_ps2_data(void) {
	static u32 systick_ms_bak = 0;
	//每50ms处理1次
	if(millis() - systick_ms_bak < 50) {
		return;
	}
	systick_ms_bak = millis();
	//读写手柄数据
	psx_write_read(psx_buf);
	
#if 0
	//测试手柄数据，1为打开 0为关闭
	sprintf((char *)cmd_return, "0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\r\n", 
	(int)psx_buf[0], (int)psx_buf[1], (int)psx_buf[2], (int)psx_buf[3],
	(int)psx_buf[4], (int)psx_buf[5], (int)psx_buf[6], (int)psx_buf[7], (int)psx_buf[8]);
	uart1_send_str(cmd_return);
#endif 	
	
	return;
}	
//处理手柄上的按钮
void loop_ps2_button(void) {
	static unsigned char psx_button_bak[2] = {0};

	//对比两次获取的按键值是否相同 ，相同就不处理，不相同则处理
	if((psx_button_bak[0] == psx_buf[3])
	&& (psx_button_bak[1] == psx_buf[4])) {				
	} else {		
		//处理buf3和buf4两个字节，这两个字节存储这手柄16个按键的状态
		parse_psx_buf(psx_buf+3, psx_buf[1]);
		psx_button_bak[0] = psx_buf[3];
		psx_button_bak[1] = psx_buf[4];
	}
	return;
}	



//--------------------------------------------------------------------------------

//软件复位函数，调用后单片机自动复位
void soft_reset(void) {
	__set_FAULTMASK(1);     
	NVIC_SystemReset();
}

//处理手柄按键字符，buf为字符数组，mode是指模式 主要是红灯和绿灯模式
void parse_psx_buf(unsigned char *buf, unsigned char mode) {
	u8 i, pos = 0;
	static u16 bak=0xffff, temp, temp2;
	temp = (buf[0]<<8) + buf[1];
	
	if(bak != temp) {
		temp2 = temp;
		temp &= bak;
		for(i=0;i<16;i++) {     //16个按键一次轮询
			if((1<<i) & temp) {
			} else {
				if((1<<i) & bak) {	//press 表示按键按下了
															
					memset(uart_receive_buf, 0, sizeof(uart_receive_buf));					
					if(mode == PS2_LED_RED){												
						//memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_red[i], strlen(pre_cmd_set_red[i]));
						if (i == 12) {
							ai_mode = 1;
							break;
						} else if (i == 14) {
							ai_mode = 2;
							break;
						} else if (i == 15) {
							ai_mode = 3;
							break;
						} else if (i == 13) {
							ai_mode = 4;
							break;
						} else if (i == 9 || i == 2) {
							ai_mode = 5;
							break;
						} else if (i == 10 || i == 3) {
							ai_mode = 6;
							break;
						} else if (i == 8 || i == 11) {
							ai_mode = 0;
							break;
						} else if (i == 4) {
							ai_mode = 11;
							break;
						} else if (i == 6) {
							ai_mode = 12;
							break;
						} else if (i == 7) {
							ai_mode = 13;
							break;
						} else if (i == 5) {
							ai_mode = 14;
							break;
						}
					}																
					pos = str_contain_str(uart_receive_buf, (u8 *)"^");
					if(pos) uart_receive_buf[pos-1] = '\0';
					if(str_contain_str(uart_receive_buf, (u8 *)"$")) {
						uart1_close();
						uart1_get_ok = 0;
						strcpy((char *)cmd_return, (char *)uart_receive_buf+11);
						strcpy((char *)uart_receive_buf, (char *)cmd_return);
						uart1_get_ok = 1;
						uart1_open();
						uart1_mode = 1;
					} else if(str_contain_str(uart_receive_buf, (u8 *)"#")) {
						uart1_close();
						uart1_get_ok = 0;
						strcpy((char *)cmd_return, (char *)uart_receive_buf+11);
						strcpy((char *)uart_receive_buf,(char *) cmd_return);
						uart1_get_ok = 1;
						uart1_open();
						uart1_mode = 2;
					}
					bak = 0xffff;
				} else {//release 表示按键松开了
										
					memset(uart_receive_buf, 0, sizeof(uart_receive_buf));					
					if(mode == PS2_LED_RED){
						//memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_red[i], strlen(pre_cmd_set_red[i]));		
						if (i == 12 || i == 14 || i == 13 || i == 15 || i == 9 || i == 2 || i == 10 || i == 3 || i == 8 || i == 11) {
							ai_mode = 0;
							break;
						}
					}										
											
					pos = str_contain_str(uart_receive_buf, (u8 *)"^");
					if(pos) {
						if(str_contain_str(uart_receive_buf+pos, (u8 *)"$")) {
							//uart1_close();
							//uart1_get_ok = 0;
							strcpy((char *)cmd_return, (char *)uart_receive_buf+pos);
							cmd_return[strlen((char *)cmd_return) - 1] = '\0';
							strcpy((char *)uart_receive_buf, (char *)cmd_return);
							parse_cmd(uart_receive_buf);
							parse_group_cmd(uart_receive_buf);
							//uart1_get_ok = 1;
							//uart1_mode = 1;
						} else if(str_contain_str(uart_receive_buf+pos, (u8 *)"#")) {
							//uart1_close();
							//uart1_get_ok = 0;
							strcpy((char *)cmd_return, (char *)uart_receive_buf+pos);
							cmd_return[strlen((char *)cmd_return) - 1] = '\0';
							strcpy((char *)uart_receive_buf, (char *)cmd_return);
							parse_action(uart_receive_buf);
							//uart1_get_ok = 1;
							//uart1_mode = 2;
						}
						//uart1_send_str(uart_receive_buf);
					}	
				}
			}
		}
		bak = temp2;
		beep_on_times(1,100);
	}	
	return;
}


//串口接收的数据解析函数
void parse_cmd(u8 *cmd) {
	int pos;//, i, index, int1, int2, int3, int4;
	//uart1_send_str(cmd);
		//智能控制相关指令，
	if(pos = str_contain_str(cmd, (u8 *)"$WAKE!"), pos){
			ai_mode = 0;	
			voice_flag = 1;
	}else if(pos = str_contain_str(cmd, (u8 *)"$TZ!"), pos){
			ai_mode = 0;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$TZ!$TZ!"), pos){
			ai_mode = 0;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$QJ!"), pos){
			ai_mode = 1;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$HT!"), pos){
			ai_mode = 2;		
	}else if(pos = str_contain_str(cmd, (u8 *)"$ZZ!"), pos){
			ai_mode = 3;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$YZ!"), pos){
			ai_mode = 4;		
	}else if(pos = str_contain_str(cmd, (u8 *)"$ZPY!"), pos){
			ai_mode = 5;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$YPY!"), pos){
			ai_mode = 6;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$DJGS!"), pos){
			ai_mode = 11;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$ZYBZ!"), pos){
			ai_mode = 12;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$ZNXJ!"), pos){
			ai_mode = 13;
	}else if(pos = str_contain_str(cmd, (u8 *)"$XJMS!"), pos){
			ai_mode = 13;
	}else if(pos = str_contain_str(cmd, (u8 *)"$XJBZ!"), pos){
			ai_mode = 14;
	}
}


//根据ai_mode的值执行对应功能
void loop_AI(void) {
	if(ai_mode == 0){
			car_run(0,0,0,0);		
			ai_mode = 99;	
	}else if(ai_mode == 1){
			car_run(600,600,600,600);
			if (voice_flag == 1) {
				tb_delay_ms(2000);
				car_run(0,0,0,0);
				voice_flag = 0;
			}
			ai_mode = 99;	
	}else if(ai_mode == 2){
			car_run(-600,-600,-600,-600);	
			if (voice_flag == 1) {
				tb_delay_ms(2000);
				car_run(0,0,0,0);
				voice_flag = 0;
			}	
			ai_mode = 99;	
	}else if(ai_mode == 3){
			car_run(-600,600,-600,600);	
			if (voice_flag == 1) {
				tb_delay_ms(500);
				car_run(0,0,0,0);
				voice_flag = 0;
			}
			ai_mode = 99;	
	}else if(ai_mode == 4){
			car_run(600,-600,600,-600);		
			if (voice_flag == 1) {
				tb_delay_ms(500);
				car_run(0,0,0,0);
				voice_flag = 0;
			}
			ai_mode = 99;	
	}else if(ai_mode == 5){
			car_run(-600,600,600,-600);		
			if (voice_flag == 1) {
				tb_delay_ms(2000);
				car_run(0,0,0,0);
				voice_flag = 0;
			}
			ai_mode = 99;	
	}else if(ai_mode == 6){
			car_run(600,-600,-600,600);		
			if (voice_flag == 1) {
				tb_delay_ms(2000);
				car_run(0,0,0,0);
				voice_flag = 0;
			}
			ai_mode = 99;	
	}else if(ai_mode == 11){
			dingju_gensui();		
	}else if(ai_mode == 12){
			ziyou_bizhang();
	}else if(ai_mode == 13){
			xun_ji();
	}else if(ai_mode == 14){
			xunji_bizhang();
	}
}


int kinematics_move(float x, float y, float z, int time) {
	int i,j, min = 0, flag = 0;
	
	if(y < 0)return 0;
	
	//寻找最佳角度
	flag = 0;
	for(i=0;i>=-135;i--) {
		if(0 == kinematics_analysis(x,y,z,i,&kinematics)){
			if(i<min)min = i;
			flag = 1;
		}
	}
	
	//用3号舵机与水平最大的夹角作为最佳值
	if(flag) {
		kinematics_analysis(x,y,z,min,&kinematics);
		for(j=0;j<4;j++) {
			set_servo(j, kinematics.servo_pwm[j], time);
		}
		return 1;
	}
	
	return 0;
}

//处理小车电机摇杆控制
void loop_ps2_car(void) {
	static int car_left, car_right, car_left_bak, car_right_bak;
	
	if(psx_buf[1] != PS2_LED_RED)return;
	
	if(abs_int(127 - psx_buf[8]) < 5 )psx_buf[8] = 127;
	if(abs_int(127 - psx_buf[6]) < 5 )psx_buf[6] = 127;
	
	//总线马达设置	
	car_left = (127 - psx_buf[8]) * 8;
	car_right = (127 - psx_buf[6]) * 8;
	
	if(abs_int(car_left) < 100)car_left = 0;
	if(car_left > 1000)car_left = 1000;
	if(car_left < -1000)car_left = -1000;
	
	if(abs_int(car_right) < 100)car_right = 0;
	if(car_right > 1000)car_right = 1000;
	if(car_right < -1000)car_right = -1000;

	if(car_left != car_left_bak || car_right != car_right_bak) {
		
		//uart1_send_str((u8*)"ps2:");
		car_run(car_left, car_right, car_left, car_right);
		car_left_bak = car_left;
		car_right_bak = car_right;
	}
}

/*************************************************************
功能介绍：发送串口指令控制电机转动
函数参数：左前轮速度，右前轮速度，左后轮速度，右后轮速度，范围：-1000~1000， 负值反转，正值正转
返回值：  无  
*************************************************************/
void car_run(int speedlq, int speedrq, int speedlh, int speedrh) {	
	sprintf((char *)cmd_return, "{#006P%04dT0000!#007P%04dT0000!#008P%04dT0000!#009P%04dT0000!}", (int)(1500+speedlq), (int)(1500-speedrq), (int)(1500+speedlh), (int)(1500-speedrh));
	zx_uart_send_str(cmd_return);
	return;
}


/***********************************************
	函数名称：loop_ir() 
	功能介绍：循环接收处理红外遥控器数据
	函数参数：无
	返回值：	无
 ***********************************************/
void loop_ir(void) {
	if(ir_flag) {
		ir_function();
		ir_flag = 0;
	}
}
