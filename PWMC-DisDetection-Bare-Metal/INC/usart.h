/**
 * @file usart.h
 * @brief USART Mode
 * including GPIO_Config.
 * 
 */

#ifndef __USART_H
#define __USART_H
#include "stdint.h"

/**
 * @brief USART MAIN CONFIG
 * 
 */
void USART_Config(void);

/**
 * @brief Send by USART2
 * 
 * @param str Pointer to the string
 */
void USART_SendString(int8_t *str);

#endif
