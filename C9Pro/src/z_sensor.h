#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "stm32f10x_conf.h"



#define AI_Read() GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)



//处理智能传感器功能
void setup_sensor(void);	//初始化所有传感器
void AI_jiaqu(void);		//静态识别夹取

#endif


