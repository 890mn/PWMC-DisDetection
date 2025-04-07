#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "stm32f10x_conf.h"

#define Trig(x) gpioB_pin_set(0, x);
#define Echo() GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2)

#define xj_l() GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)
#define xj_r() GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)

//处理智能传感器功能
void setup_sensor(void);

void setup_xunji(void);
void setup_csb(void);

void dingju_gensui(void);
void ziyou_bizhang(void);
int get_csb_value(uint8_t sensor_id);  

#endif
