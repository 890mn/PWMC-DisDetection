#ifndef __TB_GLOBAL_H__
#define __TB_GLOBAL_H__

#include "stm32f10x_conf.h"

#define PRE_CMD_SIZE 128
typedef struct {
	u32 version;
	//u32 dj_record_num;
	u8  pre_cmd[PRE_CMD_SIZE + 1];
	//int dj_bias_pwm[DJ_NUM+1];
	u8 color_base_flag;
	int color_red_base;
	int color_grn_base;
	int color_blu_base;
}eeprom_info_t;

#define CMD_RETURN_SIZE 1024
extern u8 cmd_return[CMD_RETURN_SIZE];
#define UART_BUF_SIZE 1024
extern u8 uart_receive_buf[UART_BUF_SIZE], uart1_get_ok, uart1_mode;
extern eeprom_info_t eeprom_info;

typedef struct {
    u8 mode;
    u8 buf[UART_BUF_SIZE];
    u16 index;
    u8 ready;
} UART_Frame;
extern UART_Frame uart_frame;

void tb_global_init(void);
uint16_t str_contain_str(unsigned char *str, unsigned char *str2);
int abs_int(int int1);
void selection_sort(int *a, int len);

#endif


