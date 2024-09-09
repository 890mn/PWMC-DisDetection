#ifndef __MAIN_H
#define __MAIN_H

#include "lcd.h"
#include "tim.h"
#include "nvic.h"
#include "usart.h"
#include "gpio.h"
#include "led.h"

/**
 * @brief Filter used for LCD Display in quick refresh mode.
 * 
 * This function filters the input distance value for display on an LCD in a mode
 * that prioritizes quick refresh rates.
 *
 * @param new_distance The new distance value to be filtered (float).
 * @return The filtered distance value (float).
 */
float Filter_Distance(float new_distance);

void Motor_Control(float distance);
void Update_Octagons(float distance, float pwm_pulse);
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
void Delay_Ms(uint32_t nTime);
void LCD_Back_Init(void);

#endif
