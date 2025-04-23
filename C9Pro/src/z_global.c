#include "z_global.h"
#include "stm32f10x_conf.h"

u8 cmd_return[CMD_RETURN_SIZE];
u8 uart1_get_ok, uart1_mode;
UART_Frame uart_frame = {0};

uint16_t str_contain_str(unsigned char *str, unsigned char *str2) {
	unsigned char *str_temp, *str_temp2;
	str_temp = str;
	str_temp2 = str2;
	while(*str_temp) {
		if(*str_temp == *str_temp2) {
			while(*str_temp2) {
				if(*str_temp++ != *str_temp2++) {
					str_temp = str_temp - (str_temp2-str2) + 1;
					str_temp2 = str2;
					break;
				}	
			}
			if(!*str_temp2) {
				return (str_temp-str);
			}
			
		} else {
			str_temp++;
		}
	}
	return 0;
}

int abs_int(int int1) {
	if(int1 > 0)return int1;
	return (-int1);
}
