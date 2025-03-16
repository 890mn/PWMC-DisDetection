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
void xun_ji(void);
void xunji_bizhang(void);
int get_csb_value(void);
int get_adc_csb_middle();   

#endif