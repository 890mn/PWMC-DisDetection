/*
	动作组控制相关函数
	
	指令表：
	$DST!              所有舵机停在当前位置，包括总线舵机和pwm舵机
	$DST:x!            x号舵机停在当前位置，包括总线舵机和pwm舵机
	$RST!              软件复位
	$PGP:%d-%d!        将存储器中的动作组从USB（uart1）打印出来     
	$DGS:x!            执行GXXXX一组动作组
	$DGT:%d-%d,%d!     执行x-x动作组x次
	$DJR!              所有舵机复位
	$GETA!
		
	波特率 22.1184	
*/

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
#include <math.h>		//标准库文件
#include "z_kinematics.h"	//逆运动学算法
#include "stm32f10x_iwdg.h"
#include "z_action.h"

int bias;                    //
//int i;								    //常用的一个临时变量
int do_start_index;				//动作组执行 起始序号
int do_times;						  //动作组执行 执行次数
int group_num_start;			//动作组执行 起始序号
int group_num_end;				//动作组执行 终止序号
int group_num_times;			//动作组执行 起始变量
//u32 dj_record_time = 1000;//学习时间默认1000

u8 needSaveFlag = 0;		    //偏差保存标志
//u32 save_addr_sector = 0;   
u32 bias_systick_ms_bak = 0;//偏差保存标志时间
u32 action_time = 0;        //动作组执行时间间隔

//--------------------------------------------------------------------------------
/*
	初始化函数实现
*/

//初始化舵机IO口
void setup_djio(void) {
	dj_io_init();		//舵机IO口初始化
}	

//初始化定时器2 处理舵机PWM输出
void setup_dj_timer(void) {
	//timer0_init();	//51中的定时器初始化
	TIM2_Int_Init(20000, 71);	 //32中的定时器初始化
}

/***********************************************
	函数名称：setup_servo_bias() 
	功能介绍：初始化舵机，将偏差带入初始值
	函数参数：无
	返回值：	无
 ***********************************************/
void setup_servo_bias(void) {
	u8 i;
	for(i=0;i<DJ_NUM;i++) {
		duoji_doing[i].aim = 1500+eeprom_info.dj_bias_pwm[i];
		duoji_doing[i].cur = duoji_doing[i].aim;
		duoji_doing[i].inc = 0;		
	}
}

//执行开机动作组
void setup_do_group(void) {
		u8 i;
	//执行预存命令 {G0000#000P1500T1000!#000P1500T1000!}
	if(eeprom_info.pre_cmd[PRE_CMD_SIZE] == FLAG_VERIFY) {
		strcpy((char *)uart_receive_buf, (char *)eeprom_info.pre_cmd);
		if(eeprom_info.pre_cmd[0] == '$') {
			parse_group_cmd(eeprom_info.pre_cmd);
		} else {
			for(i=16;i<strlen((char *)uart_receive_buf);i+=15) {
				uart_receive_buf[i] = '0';
				uart_receive_buf[i+1] = '0';
				uart_receive_buf[i+2] = '0';
				uart_receive_buf[i+3] = '0';
			}
			parse_action(uart_receive_buf);
		}
	}
}


//--------------------------------------------------------------------------------


//--------------------------------------------------------------------------------
/*
	主循环函数实现
*/
/***********************************************
	函数名称：loop_monitor() 
	功能介绍：设置pwm舵机偏差值
	函数参数：无
	返回值：	无
 ***********************************************/

/*
void loop_monitor(void) {
	static u32 saveTime = 3000;
	if((needSaveFlag == 1) || (millis() - bias_systick_ms_bak > saveTime)) {
		needSaveFlag = 0;
		bias_systick_ms_bak = millis();
		rewrite_eeprom();
	}	
	return;
}	

*/

/***********************************************
	函数名称：loop_action() 
	功能介绍：执行动作组指令
	函数参数：无
	返回值：	无
 ***********************************************/
void loop_action(void) {
	static u32 systick_ms_bak = 0;
	if(group_do_ok == 0) {
		if(millis() - systick_ms_bak > action_time) {
			systick_ms_bak =  millis();
			if(group_num_times != 0 && do_times == 0) {
			  group_do_ok = 1;
			  uart1_send_str((u8 *)"@GroupDone!");
			  return;
			}
			//调用do_start_index个动作
			do_group_once(do_start_index);
			
			if(group_num_start<group_num_end) {
				if(do_start_index == group_num_end) {
					do_start_index = group_num_start;
					if(group_num_times != 0) {
						do_times--;
					}
					return;
				}
				do_start_index++;
			} else {
				if(do_start_index == group_num_end) {
					do_start_index = group_num_start;
					if(group_num_times != 0) {
						do_times--;
					}
					return;
				}
				do_start_index--;
			}
		}
	} else {
      action_time = 10;
	}
}

/***********************************************
	函数名称：parse_group_cmd() 
	功能介绍：解析 $ 开头 ! 结尾的指令，处理动作组相关指令
	函数参数：字符串指令表中的指令
	返回值：	无
 ***********************************************/
void parse_group_cmd(u8 *cmd) {
	int pos, i, index, int1, int2;
	//uart1_send_str(cmd);

	if(pos = str_contain_str(cmd, (u8 *)"$DRS!"), pos) {
		
		
		uart1_send_str((u8 *)"module:");
		//uart1_send_str((u8 *)MODULE);
		uart1_send_str((u8 *)"\r\n");
		
		uart1_send_str((u8 *)"bias:");
		for(i=0;i<DJ_NUM;i++) {
	    sprintf((char*)cmd_return, "%d  ", eeprom_info.dj_bias_pwm[i]);
			uart1_send_str(cmd_return);
		}
		uart1_send_str((u8 *)"\r\n");
		
		uart1_send_str((u8 *)"preCmd:");
		uart1_send_str( eeprom_info.pre_cmd);
		
	} else if(pos = str_contain_str(cmd, (u8 *)"$DST!"), pos) {
		group_do_ok  = 1;
		for(i=0;i<DJ_NUM;i++) {
			duoji_doing[i].inc = 0;	
			duoji_doing[i].aim = duoji_doing[i].cur;
		}
		zx_uart_send_str((u8 *)"#255PDST!");//总线停止
	} else if(pos = str_contain_str(cmd, (u8 *)"$DST:"), pos) {
		if(sscanf((char *)cmd, "$DST:%d!", &index)) {
			duoji_doing[index].inc = 0;	
			duoji_doing[index].aim = duoji_doing[index].cur;
			sprintf((char *)cmd_return, "#%03dPDST!\r\n", (int)index);
			zx_uart_send_str(cmd_return);
			memset(cmd_return, 0, sizeof(cmd_return));
		}
	} else if(pos = str_contain_str(cmd, (u8 *)"$RST!"), pos) {
			//rewrite_eeprom();
			//mdelay(500);
			soft_reset();
	} else if(pos = str_contain_str(cmd, (u8 *)"$PTG:"), pos) {		
		if(sscanf((char *)cmd, "$PTG:%d-%d!", &int1, &int2)) {
			print_group(int1, int2);
		}
	} else if(pos = str_contain_str(cmd, (u8 *)"$DGS:"), pos) {		
		if(sscanf((char *)cmd, "$DGS:%d!", &int1)) {
			group_do_ok = 1;
			do_group_once(int1);
		}
	} else if(pos = str_contain_str(cmd, (u8 *)"$DGT:"), pos) {		
		if(sscanf((char *)cmd, "$DGT:%d-%d,%d!", &group_num_start, &group_num_end, &group_num_times)) {		
			if(group_num_start != group_num_end) {
				do_start_index = group_num_start;
				do_times = group_num_times;
				group_do_ok = 0;
			} else {
				do_group_once(group_num_start);
			}
		}
	} else if(pos = str_contain_str(cmd, (u8 *)"$DJR!"), pos) {	
		zx_uart_send_str((u8 *)"#255P1500T2000!\r\n");
		for(i=0;i<DJ_NUM;i++) {
			duoji_doing[i].aim = 1500+eeprom_info.dj_bias_pwm[i];
			duoji_doing[i].time = 2000;
			duoji_doing[i].inc = (duoji_doing[i].aim -  duoji_doing[i].cur) / (duoji_doing[i].time/20.000);
		}		
	} else if(pos = str_contain_str(cmd, (u8 *)"$GETA!"), pos) {		
			uart1_send_str((u8 *)"AAA");
	} 
}


/***********************************************
	函数名称：save_action() 
  功能介绍：动作组保存函数，设置开机动作组
	函数参数：只有用<>包含的字符串才能在此函数中进行解析
	返回值：	无
 ***********************************************/
void save_action(u8 *str) {
	int action_index = 0;
	//预存命令处理
	spiFlahsOn(1);
	if(str[1] == '$' && str[2] == '!') {
		eeprom_info.pre_cmd[PRE_CMD_SIZE] = 0;
		rewrite_eeprom();   //去掉开机动作组
		uart1_send_str((u8 *)"@CLEAR PRE_CMD OK!");
		return;
	}else if(str[1] == '$') {
		//设置开机动作组
		if(sscanf((char *)str, "<$DGT:%d-%d,%d!>", &group_num_start, &group_num_end, &group_num_times)) {
			if(group_num_start == group_num_end) {
				w25x_read(eeprom_info.pre_cmd, group_num_start*ACTION_SIZE, ACTION_SIZE);	
			} else {
				memset(eeprom_info.pre_cmd, 0, sizeof(eeprom_info.pre_cmd));
				strcpy((char *)eeprom_info.pre_cmd, (char *)str+1);
				eeprom_info.pre_cmd[strlen((char *)str) - 2] = '\0';
			}
			eeprom_info.pre_cmd[PRE_CMD_SIZE] = FLAG_VERIFY;
			rewrite_eeprom();
			//uart1_send_str(eeprom_info.pre_cmd);
			uart1_send_str((u8 *)"@SET PRE_CMD OK!");
		}
		return;
	}
	
	action_index = get_action_index(str);
	//<G0001#001...>
	if((action_index == -1) || str[6] != '#'){
	//if( action_index == -1 ){
		uart1_send_str("E");
		return;
	}
	//save_action_index_bak++;
	//把尖括号替换成大括号直接存储到存储芯片里面去，则在执行动作组的时候直接拿出来解析就可以了
	replace_char(str, '<', '{');
	replace_char(str, '>', '}');
	
	if(action_index*ACTION_SIZE % W25Q64_SECTOR_SIZE == 0)w25x_erase_sector(action_index*ACTION_SIZE/W25Q64_SECTOR_SIZE);

	w25x_write(str, action_index*ACTION_SIZE, strlen(str) + 1);
	//uart1_send_str(str);
	uart1_send_str("A");
	spiFlahsOn(0);
	return;	
}

/***********************************************
	函数名称：get_action_index() 
	功能介绍：获取动作组序号
	函数参数：无
	返回值：	无
 ***********************************************/
int get_action_index(u8 *str) {
	int index = 0;
	//uart_send_str(str);
	while(*str) {
		if(*str == 'G') {
			str++;
			while((*str != '#') && (*str != '$')) {
				index = index*10 + *str-'0';
				str++;	
			}
			return index;
		} else {
			str++;
		}
	}
	return -1;
}

/***********************************************
	函数名称：print_group() 
	功能介绍：打印存储器中的动作组
	函数参数：起始位置，终止位置
	返回值：	无
 ***********************************************/
void print_group(int start, int end) {
	spiFlahsOn(1);
	
	if(start > end) {
		int_exchange(&start, &end);
	}
	
	for(;start<=end;start++) {
		memset(uart_receive_buf, 0, sizeof(uart_receive_buf));
		w25x_read(uart_receive_buf, start*ACTION_SIZE, ACTION_SIZE);
		uart1_send_str(uart_receive_buf);
		uart1_send_str((u8 *)"\r\n");
		
		uart3_send_str(uart_receive_buf);
		uart3_send_str((u8 *)"\r\n");
	}
	
	spiFlahsOn(0);
}

/***********************************************
	函数名称：int_exchange() 
	功能介绍：数据交换
	函数参数：无
	返回值：	无
 ***********************************************/
void int_exchange(int *int1, int *int2) {
	int int_temp;
	int_temp = *int1;
	*int1 = *int2;
	*int2 = int_temp;
}

/***********************************************
	函数名称：erase_sector() 
	功能介绍：擦除动作组指令
	函数参数：无
	返回值：	无
 ***********************************************/
/*
void erase_sector(int start, int end) {
	if(start > end) {
		int_exchange(&start, &end);
	}
	if(end >= 127)end = 127;
	for(;start<=end;start++) {
		SpiFlashEraseSector(start);
		sprintf(cmd_return, "@Erase %d OK!", start);
		uart1_send_str(cmd_return);
	}
	save_action_index_bak = 0;
}
*/

/***********************************************
	函数名称：getMaxTime() 
	功能介绍：获取最大时间
	函数参数：无
	返回值：	无
 ***********************************************/
int getMaxTime(u8 *str) {
   int i = 0, max_time = 0, tmp_time = 0;
   while(str[i]) {
      if(str[i] == 'T') {
          tmp_time = (str[i+1]-'0')*1000 + (str[i+2]-'0')*100 + (str[i+3]-'0')*10 + (str[i+4]-'0');
          if(tmp_time>max_time)max_time = tmp_time;
          i = i+4;
          continue;
      }
      i++;
   }
   return max_time;
}

/***********************************************
	函数名称：do_group_once() 
	功能介绍：从存储芯片中读取第group_num个动作组，执行动作组
	函数参数：动作组序号
	返回值：	无
 ***********************************************/
void do_group_once(int group_num) {
	spiFlahsOn(1);
	//将uart_receive_buf清零
	memset(uart_receive_buf, 0, sizeof(uart_receive_buf));
	//从存储芯片中读取第group_num个动作组
	w25x_read(uart_receive_buf, group_num*ACTION_SIZE, ACTION_SIZE-1);	
	//获取最大的组时间
	action_time = getMaxTime(uart_receive_buf);
	
	//把读取出来的动作组传递到parse_action执行
	parse_action(uart_receive_buf);
	spiFlahsOn(0);
}

/***********************************************
	函数名称：parse_action() 
	功能介绍：解析以#开头！结尾的指令
	函数参数：无
	返回值：	无
 ***********************************************/
void parse_action(u8 *uart_receive_buf) {
	u16 index, time, i = 0, j, step = 0;
	float pwm;
	float aim_temp;
	zx_uart_send_str(uart_receive_buf);
	//解析#000PSCK+001!指令，调节pwm舵机偏差
	if(uart_receive_buf[0] == '#' && uart_receive_buf[4] == 'P' && uart_receive_buf[5] == 'S' && uart_receive_buf[6] == 'C' && uart_receive_buf[7] == 'K' && uart_receive_buf[12] == '!') {
		index = (uart_receive_buf[1] - '0')*100 + (uart_receive_buf[2] - '0')*10 + (uart_receive_buf[3] - '0');
		bias = (uart_receive_buf[9] - '0')*100 + (uart_receive_buf[10] - '0')*10 + (uart_receive_buf[11] - '0');
		if((bias >= -500) && (bias <= 500) && (index < DJ_NUM)) {
			if(uart_receive_buf[8] == '+') {
			} else if(uart_receive_buf[8] == '-') {
				bias = -bias;
			}
				
			aim_temp = duoji_doing[index].cur - (eeprom_info.dj_bias_pwm[index] - bias);
			eeprom_info.dj_bias_pwm[index] = bias; //将偏差记录到eeprom中
			//bias_bak = bias;
			if(aim_temp > 2497){
				aim_temp = 2497;
			} else if(aim_temp < 500) {
				aim_temp = 500;
			}
			//转动舵机，可看出当前偏差
			duoji_doing[index].aim = aim_temp;
			duoji_doing[index].inc = (duoji_doing[index].aim - duoji_doing[index].cur)/5;
			rewrite_eeprom();			
		}
		return;
		//解析#000PDST!指令，使PWM舵机停止在当前位置
	} else if(uart_receive_buf[0] == '#' && uart_receive_buf[4] == 'P' && uart_receive_buf[5] == 'D' && uart_receive_buf[6] == 'S' && uart_receive_buf[7] == 'T' && uart_receive_buf[8] == '!') {
		index = (uart_receive_buf[1] - '0')*100 + (uart_receive_buf[2] - '0')*10 + (uart_receive_buf[3] - '0');		
		if(index < DJ_NUM) {
			duoji_doing[index].inc = 0;	
			duoji_doing[index].aim = duoji_doing[index].cur;
		}
		return;
	}
	
	step = 1;
	while(uart_receive_buf[i]) {
		if(uart_receive_buf[i] == '#' && step == 1) {
			j = i;
			index = 0;i++;
			while(uart_receive_buf[i] && uart_receive_buf[i] != 'P') {
				index = index*10 + uart_receive_buf[i]-'0';i++;
			}
			if(i-j-1 != 3) {
				step = 1;
			} else {
				step = 2;
			}
		} else if(uart_receive_buf[i] == 'P' && step == 2) {
			j = i;
			pwm = 0;i++;
			while(uart_receive_buf[i] && uart_receive_buf[i] != 'T') {
				pwm = pwm*10 + uart_receive_buf[i]-'0';i++;
			}
			if(i-j-1 != 4) {
				step = 1;
			} else {
				step = 3;
			}
		} else if(uart_receive_buf[i] == 'T' && step == 3) {
			j = i;
			time = 0;i++;
			while(uart_receive_buf[i] && uart_receive_buf[i] != '!') {
				time = time*10 + uart_receive_buf[i]-'0';i++;
			}
			
			//同步的时候防止数据太快不稳定
			if(time<500) {
				time = time+300;
			}
			
			step = 1;
			if(i-j-1 != 4) {
			} else {
				if(index < DJ_NUM && (pwm<=2500)&& (pwm>=500) && (time<=10000)) {
					pwm += eeprom_info.dj_bias_pwm[index];
					
					if(pwm>2497)pwm=2497;
					if(pwm<500)pwm=500;
					
					if(time < 20) {
						duoji_doing[index].aim = pwm;
						duoji_doing[index].cur = pwm;
						duoji_doing[index].inc = 0;
					} else {
						duoji_doing[index].aim = pwm;
						duoji_doing[index].time = time;
						duoji_doing[index].inc = (duoji_doing[index].aim -  duoji_doing[index].cur) / (duoji_doing[index].time/20.000);
					}

					//sprintf(cmd_return, "#%03dP%04dT%04d! %f \r\n", index, pwm, time, duoji_doing[index].inc);
					//uart1_send_str(cmd_return);
				} 
				
				if(index == 255) {
					for(index=0;index<DJ_NUM;index++) {
						pwm =1500 + eeprom_info.dj_bias_pwm[index];
						duoji_doing[index].aim = pwm;
						duoji_doing[index].time = time;
						duoji_doing[index].inc = (duoji_doing[index].aim -  duoji_doing[index].cur) / (duoji_doing[index].time/20.000);
					}
				}
			}
		} else {
			i++;
		}
	}	
}

/***********************************************
	函数名称：replace_char() 
	功能介绍：字符串中的字符替代函数 把str字符串中所有的ch1换成ch2
	函数参数：字符串
	返回值：	无
 ***********************************************/
void replace_char(u8*str, u8 ch1, u8 ch2) {
	while(*str) {
		if(*str == ch1) {
			*str = ch2;
		} 
		str++;
	}
	return;
}

/***********************************************
	函数名称：rewrite_eeprom() 
	功能介绍：把eeprom_info写入到W25Q64_INFO_ADDR_SAVE_STR位置
	函数参数：无
	返回值：	无
 ***********************************************/
void rewrite_eeprom(void) {
	spiFlahsOn(1);
	w25x_erase_sector(W25Q64_INFO_ADDR_SAVE_STR/4096);
	w25x_writeS((u8 *)(&eeprom_info), W25Q64_INFO_ADDR_SAVE_STR, sizeof(eeprom_info));
	spiFlahsOn(0);
}

/***********************************************
	函数名称：set_servo() 
	功能介绍：舵机控制
	函数参数：id，pwm值，转动时间
	返回值：	无
 ***********************************************/
void set_servo(int index, int pwm, int time) {
	duoji_doing[index].aim = pwm;
	duoji_doing[index].time = time;
	duoji_doing[index].inc = (duoji_doing[index].aim -  duoji_doing[index].cur) / (duoji_doing[index].time/20.000);
	sprintf((char *)cmd_return, "#%03dP%04dT%04d!\r\n", index, pwm, time);
	zx_uart_send_str(cmd_return);	
}
