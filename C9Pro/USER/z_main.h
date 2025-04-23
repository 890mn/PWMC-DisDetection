#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f10x_conf.h"
#include "z_timer.h"
#include "z_rcc.h"		//配置时钟文件
#include "z_gpio.h"		//配置IO口文件
#include "z_delay.h"	//存放延时函数
#include "z_type.h"		//存放类型定义
#include "z_usart.h"	//存放串口功能文件
#include "z_timer.h"	//存放定时器功能文件
#include <stdio.h>		//标准库文件
#include <string.h>		//标准库文件
#include <math.h>	  	//标准库文件
#include <stdlib.h>
#include <ctype.h>
#include "stm32f10x_iwdg.h"
#include "z_sensor.h"

#define SENSOR_LEFT   1
#define SENSOR_FRONT  2
#define SENSOR_RIGHT  3
#define SENSOR_BACK   4

#define DIST(t) ( (6 * (t) * (t)) / 1000 + 25 * (t) - 5500 )    // mm
#define ANGT(t) ( (-5 * (t) * (t)) / 1000 + 203 * (t) - 62750 ) // mdeg

// distance -> time (ms)，反向估算执行时间
#define TIME_FROM_DIST(d) (20 * (d) + 220000)  // 粗略拟合，保证时间合理区间
#define TIME_FROM_ANGT(a) (15 * (a) + 180000) // 角度转时间的估算

#define DEFAULT_TIME_MS  1000
#define SHORT_TIME_MS    500
#define TURN_TIME_MS     800

typedef enum {
    DIR_STOP = 0,
    DIR_FORWARD,
    DIR_BACK,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_LEFT_FORWARD,
    DIR_RIGHT_FORWARD,
    DIR_RIGCEN,
    DIR_RIGCEN_REV,
    DIR_LEFCEN,
    DIR_LEFCEN_REV
} Direction;

typedef struct {
    const char* cmd;
    Direction dir;
} CommandMap;

extern CommandMap directionTable[];
extern Direction main_direction;
extern uint16_t  main_distance;
extern uint16_t  ultra_left;
extern uint16_t  ultra_front;
extern uint16_t  ultra_right;
extern uint16_t  ultra_back;

void setup_rcc(void);         //初始化时钟
void setup_gpio(void);        //初始化IO口			
void setup_nled(void);        //初始化工作指示灯
void setup_beep(void);	      //初始化蜂鸣器		
void setup_uart1(void);	      //初始化串口1
void setup_uart3(void);	      //初始化串口3
void setup_systick(void);     //初始化滴答时钟，1S增加一次systick_ms的值
void setup_interrupt(void);   //初始化总中断

void loop_nled(void);	        //循环执行工作指示灯，500ms跳动一次	
void loop_uart(void);	        //串口数据接收处理

void soft_reset(void);
void parse_cmd(u8 *cmd);

void car_run(int speedlq, int speedrq, int speedlh, int speedrh, int duration_ms);
int get_default_time(Direction dir);
void execute_direction(Direction dir, int duration_ms);

Direction get_direction_from_str(const char* dirStr);
void ultra_distance(void);
void avoid_system(void);
void dirs(u8 *cmd);

#endif

