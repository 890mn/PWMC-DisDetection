#ifndef __ACTION_H__
#define __ACTION_H__

#define CAR_PWM						0
#define ADC_VOL						7
#define ACTION_SIZE					256
#define W25Q64_INFO_ADDR_SAVE_STR	(((8<<10)-4)<<10)//(8*1024-4)*1024		//eeprom_info结构体存储的位置
#define FLAG_VERIFY 				0x38

#include "stm32f10x_conf.h"

/*
	初始化函数声明
*/
			
//初始化低压报警
void setup_vol(void);		
//初始化舵机IO口
void setup_djio(void);			
//初始化定时器2 处理舵机PWM输出
void setup_dj_timer(void);		
			
//初始化舵机偏差
void setup_servo_bias(void);
//执行开机动作组
void setup_do_group(void);

//动作组批量执行
void loop_action(void);		
//定时保存偏差变量
void loop_monitor(void);	

/*
	子循环函数声明
*/
void parse_group_cmd(u8 *cmd);

void save_action(u8 *str);
int get_action_index(u8 *str);//获取动作序号
void print_group(int start, int end);
void int_exchange(int *int1, int *int2);
void erase_sector(int start, int end);

void do_group_once(int group_num); 
u8 check_dj_state(void);//检查舵机状态，是否全部到位

void parse_action(u8 *uart_receive_buf);
void replace_char(u8*str, u8 ch1, u8 ch2);
void rewrite_eeprom(void);

void set_servo(int index, int pwm, int time);	//设置舵机角度					
#endif 