#ifndef __LED_H
#define __LED_H

#include "stdint.h"

//CT117E LED
#define LED0    GPIO_Pin_8
#define LED1    GPIO_Pin_9
#define LED2    GPIO_Pin_10
#define LED3    GPIO_Pin_11
#define LED4    GPIO_Pin_12
#define LED5    GPIO_Pin_13
#define LED6    GPIO_Pin_14
#define LED7    GPIO_Pin_15
#define LEDALL	GPIO_Pin_All

/**
 * @brief Control the LED
 * 
 * @param LED DEFINE LED0-LED7 | LEDALL
 * @param LED_Status 0 Close | 1 Open
 */
void LED_Control(uint16_t LED,uint8_t LED_Status);

/**
 * @brief Use LED to simulate PWM Duty Cycle value
 * 
 * @param pwm_pulse Duty Cycle
 */
void LED_PWM(float pwm_pulse);

#endif
