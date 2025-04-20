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
#include "z_sensor.h"
#include "z_action.h"
#include "z_main.h"

u16 kms_y = 0;
u8 get_count = 0;
u8 carry_step = 0;

/*
	辅助控 S2  TRIG - PA0 - 白线  Echo - PA1 - 绿线
	传感器 S3  TRIG - PB0 - 白线  Echo - PA2 - 绿线
	传感器 S4  TRIG - PB1 - 白线  Echo - PA3 - 绿线
	传感器 S5  TRIG - PA6 - 白线  Echo - PA4 - 绿线
	传感器 S6  TRIG - PA7 - 白线  Echo - PA5 - 绿线
		
*/
uint16_t trig_pins[] = {GPIO_Pin_0, GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_6, GPIO_Pin_7};
uint16_t echo_pins[] = {GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3, GPIO_Pin_4, GPIO_Pin_5};
GPIO_TypeDef *trig_ports[] = {GPIOA, GPIOB, GPIOB, GPIOA, GPIOA};
GPIO_TypeDef *echo_ports[] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOA};

//初始化传感器IO口
void setup_sensor(void) {
	setup_xunji();
	setup_csb();
}

void setup_csb(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);  
	
	for (i = 0; i < 5; i++)
	{
		// 配置Trig引脚
		GPIO_InitStructure.GPIO_Pin = trig_pins[i];
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(trig_ports[i], &GPIO_InitStructure);

		// 配置Echo引脚
		GPIO_InitStructure.GPIO_Pin = echo_pins[i];
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(echo_ports[i], &GPIO_InitStructure);
	}
	
	//初始化超声波定时器
	TIM3_Int_Init(30000, 71);
}

 // 循迹左 A0  A1  循迹右 B0 A2
void setup_xunji(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   
	GPIO_Init(GPIOA, &GPIO_InitStructure); 	
}

void csb_Delay_Us(uint16_t time)  //延时函数
{ 
	uint16_t i,j;
	for(i=0;i<time;i++)
  		for(j=0;j<9;j++);
}

/*************************************************************
函数名称：get_csb_value()
功能介绍：采集超声波数据
函数参数：无
返回值：  采集的数据  
*************************************************************/
int get_csb_value(uint8_t sensor_id) {
    uint16_t echo_time = 0;
    long timeout = 0;
    int distance = 0;

    // 发出 TRIG 脉冲
    GPIO_SetBits(trig_ports[sensor_id], trig_pins[sensor_id]);
    csb_Delay_Us(10);
    GPIO_ResetBits(trig_ports[sensor_id], trig_pins[sensor_id]);

    // 等待 ECHO 拉高（超时保护）
    timeout = 0;
    while (GPIO_ReadInputDataBit(echo_ports[sensor_id], echo_pins[sensor_id]) == 0) {
        if (++timeout > 30000) goto send_result; // 超时，跳转发送0
    }

    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);

    // 等待 ECHO 拉低（结束测量）
    timeout = 0;
    while (GPIO_ReadInputDataBit(echo_ports[sensor_id], echo_pins[sensor_id]) == 1) {
        if (++timeout > 60000) {
            TIM_Cmd(TIM3, DISABLE);
            goto send_result; // 超时，跳转发送0
        }
    }

    TIM_Cmd(TIM3, DISABLE);
    echo_time = TIM_GetCounter(TIM3);

    // 计算距离（cm）
    distance = (int)(echo_time * 0.017f + 0.5f);

    // 合理值范围检测
    if (distance <= 0 || distance > 600) {
        distance = 0;
    }

send_result:
    // 必发串口信息
    sprintf((char *)cmd_return, "[%d]UTime=%d", sensor_id, distance);
    uart3_send_str(cmd_return);

    return distance;
}


/*************************************************************
功能介绍：定距跟随：判断超声波检测的距离，小于20cm时后退；25-35cm或超过70cm时停止；40-60cm时前进
函数参数：无
返回值：  无
*************************************************************/
void dingju_gensui(){
	static u32 systick_ms_bak = 0;
	int adc_csb;
	if(millis() - systick_ms_bak > 100) {
		systick_ms_bak = millis();
		//adc_csb = get_adc_csb_middle();             //获取a0的ad值，计算出距离
		if(adc_csb < 20) {                          //距离小于20cm时后退
			car_run(-600,-600,-600,-600);
			tb_delay_ms(100);
		} else if ((25 < adc_csb && adc_csb < 35) || adc_csb > 70) {   //25-35cm或超过70cm时停止
			car_run(0,0,0,0);
			tb_delay_ms(100);
		} else if (40 < adc_csb && adc_csb < 60) {  //40-60cm时前进
			car_run(600,600,600,600);
			tb_delay_ms(100);
		}
	}
}


/*************************************************************
功能介绍：自由避障：判断超声波检测的距离，小于20cm时右转避障，大于20cm时前进
函数参数：无
返回值：  无
*************************************************************/
void ziyou_bizhang(){
	static u32 systick_ms_bak = 0;
    int dist_front, dist_left, dist_right, dist_back;

    if (millis() - systick_ms_bak > 100) { // 每 100ms 进行一次避障判断
        systick_ms_bak = millis();

        // 读取四个方向超声波数据
        dist_front = get_csb_value(1); // S3 - 前方
        dist_left  = get_csb_value(2); // S4 - 左方
        dist_right = get_csb_value(3); // S5 - 右方
        dist_back  = get_csb_value(4); // S6 - 后方

        if (dist_front >= 20) {
            // 前方无障碍，继续前进
            car_run(600, 600, 600, 600);
        } 
        else {
            // 前方有障碍，选择左右方向
            if (dist_left >= 20 && dist_right >= 20) {
                // 左右都可行，选择距离更远的方向
                if (dist_left > dist_right) {
                    car_run(-600, 600, -600, 600); // 左转
                } else {
                    car_run(600, -600, 600, -600); // 右转
                }
            } 
            else if (dist_left >= 20) {
                car_run(-600, 600, -600, 600); // 左转
            } 
            else if (dist_right >= 20) {
                car_run(600, -600, 600, -600); // 右转
            } 
            else {
                // 四向均被挡住，尝试后退
                if (dist_back >= 20) {
                    car_run(-600, -600, -600, -600); // 后退
                    tb_delay_ms(500); // 退一段时间
                } else {
                    car_run(0, 0, 0, 0); // 停止
                }
            }
            tb_delay_ms(500); // 避障等待时间
        }
    }
}
