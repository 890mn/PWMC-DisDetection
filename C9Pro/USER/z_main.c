#include "z_main.h"

uint16_t ultra_left = 0;
uint16_t ultra_front = 0;
uint16_t ultra_right = 0;
uint16_t ultra_back = 0;

uint16_t main_distance = 0;
Direction main_direction = DIR_STOP;

bool avoid_mode = FALSE;
bool any = FALSE;

CommandMap directionTable[] = {
    {"Stop", DIR_STOP},
    {"Forward", DIR_FORWARD},
    {"Back", DIR_BACK},
    {"Left", DIR_LEFT},
    {"Right", DIR_RIGHT},
    {"LeftForward", DIR_LEFT_FORWARD},
    {"RightForward", DIR_RIGHT_FORWARD},
    {"RigCen", DIR_RIGCEN},
    {"RigCenRev", DIR_RIGCEN_REV},
    {"LefCen", DIR_LEFCEN},
    {"LefCenRev", DIR_LEFCEN_REV}
};

void execute_direction(Direction dir) {
	if (any) {
		car_run(0, 0, 0, 0);
	} else {
		switch (dir) {
			case DIR_STOP:           car_run(0, 0, 0, 0); break;
			case DIR_FORWARD:        car_run(600, 600, 600, 600); break;
			case DIR_BACK:           car_run(-600, -600, -600, -600); break;
			case DIR_LEFT:           car_run(-600, 600, -600, 600); break;
			case DIR_RIGHT:          car_run(600, -600, 600, -600); break;
			case DIR_LEFT_FORWARD:   car_run(-600, 600, 600, -600); break;
			case DIR_RIGHT_FORWARD:  car_run(600, -600, -600, 600); break;
			case DIR_RIGCEN:         car_run(600, 0, 600, 0); break;
			case DIR_RIGCEN_REV:     car_run(-600, 0, -600, 0); break;
			case DIR_LEFCEN:         car_run(0, -600, 0, -600); break;
			case DIR_LEFCEN_REV:     car_run(0, 600, 0, 600); break;
			default:                 car_run(0, 0, 0, 0); break;
		}
	}
}

Direction get_direction_from_str(const char* dirStr) {
    for (int i = 0; i < sizeof(directionTable)/sizeof(CommandMap); ++i) {
        if (strcmp(dirStr, directionTable[i].cmd) == 0) {
            return directionTable[i].dir;
        }
    }
    return DIR_STOP;
}

void ultra_distance(void) {
    ultra_left = get_csb_value(SENSOR_LEFT);
    tb_delay_ms(5); // 左-前之间稍作等待

    ultra_front = get_csb_value(SENSOR_FRONT);
    tb_delay_ms(5);

    ultra_right = get_csb_value(SENSOR_RIGHT);
    tb_delay_ms(5);

    ultra_back = get_csb_value(SENSOR_BACK);
    tb_delay_ms(10);
}

void dirs(u8 *cmd) {
	char directionStr[20] = {0};
    char numberStr[10] = {0};
    int i = 1, j = 0;
    while (isalpha(cmd[i]) && j < sizeof(directionStr) - 1) {
        directionStr[j++] = cmd[i++];
    }
    directionStr[j] = '\0';
    j = 0;
    while (isdigit(cmd[i]) && j < sizeof(numberStr) - 1) {
        numberStr[j++] = cmd[i++];
    }
    numberStr[j] = '\0';
    main_direction = get_direction_from_str(directionStr);
    main_distance = atoi(numberStr);
}

void avoid_system(void) {
    if (!avoid_mode) return;

    int safe = 0;
	any = FALSE;
    // ------- 1. 判断目标方向是否畅通 -------
    switch (main_direction) {
        case DIR_FORWARD:
            if (ultra_front > main_distance + 10) {
                execute_direction(main_direction);
                safe = 1;
            }
            break;
        case DIR_BACK:
            if (ultra_back > main_distance + 10) {
                execute_direction(main_direction);
                safe = 1;
            }
            break;
        case DIR_LEFT:
            if (ultra_left > main_distance + 10) {
                execute_direction(main_direction);
                safe = 1;
            }
            break;
        case DIR_RIGHT:
            if (ultra_right > main_distance + 10) {
                execute_direction(main_direction);
                safe = 1;
            }
            break;
        default:
            break;
    }

    // ------- 2. 如果目标方向不安全，尝试绕行 -------
    if (!safe) {
        if (ultra_left > ultra_right && ultra_left > 20) {
            main_direction = DIR_LEFT;
        } else if (ultra_right > 20) {
            main_direction = DIR_RIGHT;
        } else if (ultra_back > 20) {
            main_direction = DIR_BACK;
        } else {
            main_direction = DIR_STOP;
        }
    }

    // ------- 3. 判断是否到达目标位置，退出避障 -------
    switch (main_direction) {
        case DIR_FORWARD:
            if (ultra_front <= main_distance + 5) {
                avoid_mode = FALSE;
                main_direction = DIR_STOP;
                zx_uart_send_str((u8*)"[避障] 前进目标已达，退出自动避障\n");
            }
            break;
        case DIR_BACK:
            if (ultra_back <= main_distance + 5) {
                avoid_mode = FALSE;
                main_direction = DIR_STOP;
                zx_uart_send_str((u8*)"[避障] 后退目标已达，退出自动避障\n");
            }
            break;
        case DIR_LEFT:
            if (ultra_left <= main_distance + 5) {
                avoid_mode = FALSE;
                main_direction = DIR_STOP;
                zx_uart_send_str((u8*)"[避障] 左移目标已达，退出自动避障\n");
            }
            break;
        case DIR_RIGHT:
            if (ultra_right <= main_distance + 5) {
                avoid_mode = FALSE;
                main_direction = DIR_STOP;
                zx_uart_send_str((u8*)"[避障] 右移目标已达，退出自动避障\n");
            }
            break;
        default:
            break;
    }
}

int main(void) {	
	setup_rcc();		  //初始化时钟
	setup_gpio();		  //初始化IO口
	setup_nled();		  //初始化工作指示灯
	setup_beep();		  //初始化定时器
	setup_uart1();		//初始化串口1 用于下载动作组
	setup_uart3();		//初始化串口3 用于底板总线、蓝牙、lora
	setup_systick();	//初始化滴答时钟，1S增加一次millis()的值
	setup_interrupt();//初始化总中断		
	IWDG_Init();       //初始化独立看门狗
	setup_sensor();
	
	tb_delay_ms(2000);
	//car_run(1000, 1000, 1000, 1000);
	zx_uart_send_str("{#006P1000T2000!#007P1000T2000!#008P1000T2000!#009P1000T2000!}");

	while(1) {
		// 1 - Base scan
		loop_nled();	   //循环执行工作指示灯，500ms跳动一次
		//ultra_distance();	
		//tb_delay_ms(350);

		// 2 - UART scan
		//loop_uart();	  //串口数据接收处理
		//avoid_system();
		//tb_delay_ms(150); // 避障系统处理

		// 3 - Car run
		//execute_direction(main_direction);
		//tb_delay_ms(500);
	}
}

//--------------------------------------------------------------------------------
/*
	初始化函数实现
*/

void setup_rcc(void) {   //初始化时钟
	tb_rcc_init();	  	   //时钟初始化
}

void setup_gpio(void) {  //初始化IO口
	tb_gpio_init();		    
}

void setup_nled(void) {  //初始化工作指示灯
	nled_init();		
	nled_off();		         //工作指示灯关闭
}

void setup_beep(void) {  //初始化定时器蜂鸣器
	beep_init();		
	beep_off();			       //关闭蜂鸣器
}			

void setup_uart1(void) {
  //串口1初始化
	tb_usart1_init(115200);
	//串口1打开
	uart1_open();
	//串口发送测试字符
	uart1_send_str((u8 *)"uart1 check ok!");
}

//初始化串口3
void setup_uart3(void) {
	//串口3初始化
	tb_usart3_init(115200);
	//串口3打开
	uart3_open();
	//串口发送测试字符
	uart3_send_str((u8 *)"uart3 check ok!");
	//总线输出 复位总线舵机 串口3即为总线串口
	zx_uart_send_str((u8 *)"#255P1500T2000!");
}	
//初始化滴答时钟，1S增加一次millis()的值
void setup_systick(void) {
	//系统滴答时钟初始化	
	SysTick_Int_Init();
}	

//初始化总中断
void setup_interrupt(void) {
	//总中断打开
	tb_interrupt_open();
}	

/*
	主循环函数实现
*/
//循环执行工作指示灯，500ms跳动一次
void loop_nled(void) {
	static u32 time_count=0;
	static u8 flag = 0;
	if(millis()-time_count > 1000)  {
		time_count = millis();
		if(flag) {
			nled_on();
		} else {
			nled_off();
		}
		flag= ~flag;
	}
}		
//串口数据接收处理
void loop_uart(void) {
	if(uart1_get_ok) {
		if(uart1_mode == 1) { // $
			beep_on_times(1,100);
			any = FALSE;
			parse_cmd(uart_receive_buf);			
		} else if(uart1_mode == 5) { // @
			beep_on_times(2,100);
			any = FALSE;
			avoid_mode = TRUE;
			dirs(uart_receive_buf);
		} 
		uart1_mode = 0;
		uart1_get_ok = 0;
		uart1_open();
	}
	return;
}	

//软件复位函数，调用后单片机自动复位
void soft_reset(void) {
	__set_FAULTMASK(1);     
	NVIC_SystemReset();
}

//串口接收的数据解析函数
void parse_cmd(u8 *cmd) {
	int pos;
	if(pos = str_contain_str(cmd, (u8 *)"$STOP!"), pos){
		main_direction = DIR_STOP;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$FORW!"), pos){
		main_direction = DIR_FORWARD;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$BACK!"), pos){
		main_direction = DIR_BACK;		
	}else if(pos = str_contain_str(cmd, (u8 *)"$LEFT!"), pos){
		main_direction = DIR_LEFT;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$RIGH!"), pos){
		main_direction = DIR_RIGHT;		
	}else if(pos = str_contain_str(cmd, (u8 *)"$LEFR!"), pos){
		main_direction = DIR_LEFT_FORWARD;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$LEBA!"), pos){
		main_direction = DIR_RIGHT_FORWARD;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$RICE!"), pos){
		main_direction = DIR_RIGCEN;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$RICR!"), pos){
		main_direction = DIR_RIGCEN_REV;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$LECE!"), pos){
		main_direction = DIR_LEFCEN;
	}else if(pos = str_contain_str(cmd, (u8 *)"$LECR!"), pos){
		main_direction = DIR_LEFCEN_REV;
	}else if(pos = str_contain_str(cmd, (u8 *)"$STOPANY!"), pos){
		main_direction = DIR_STOP;
		any = TRUE;
		avoid_mode = FALSE;
		beep_on_times(1,200);
	}
}

/*************************************************************
功能介绍：发送串口指令控制电机转动
函数参数：左前轮速度，右前轮速度，左后轮速度，右后轮速度，范围：-1000~1000， 负值反转，正值正转
返回值：  无  
*************************************************************/
void car_run(int speedlq, int speedrq, int speedlh, int speedrh) {	
	sprintf((char *)cmd_return, "{#006P%04dT0000!#007P%04dT0000!#008P%04dT0000!#009P%04dT0000!}", (int)(1500+speedlq), (int)(1500-speedrq), (int)(1500+speedlh), (int)(1500-speedrh));
	zx_uart_send_str(cmd_return);
	return;
}
