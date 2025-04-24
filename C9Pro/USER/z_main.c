#include "z_main.h"

uint16_t ultra_left = 0;
uint16_t ultra_front = 0;
uint16_t ultra_right = 0;
uint16_t ultra_back = 0;

uint16_t main_distance = 0;
Direction main_direction = DIR_STOP;

bool avoid_mode = FALSE;
bool any = FALSE;
volatile bool executed_in_this_loop = FALSE;

typedef enum {
    AVOID_IDLE,
    AVOID_PLANNING,
    AVOID_EXECUTING,
    AVOID_CHECKING
} AvoidState;

AvoidState avoid_state = AVOID_IDLE;
Direction avoid_direction = DIR_STOP;
int avoid_target_distance = 0;
int avoid_time_remaining = 0;
int avoid_distance_done = 0;
#define SAFE_THRESHOLD 20  // cm
int forward_done = 0;      // 已完成的主方向行进距离

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

int get_default_time(Direction dir) {
	switch (dir) {
		case DIR_FORWARD:
		case DIR_BACK:
			return DEFAULT_TIME_MS;
		case DIR_LEFT:
		case DIR_RIGHT:
		case DIR_LEFT_FORWARD:
		case DIR_RIGHT_FORWARD:
			return SHORT_TIME_MS;
		case DIR_RIGCEN:
		case DIR_RIGCEN_REV:
		case DIR_LEFCEN:
		case DIR_LEFCEN_REV:
			return TURN_TIME_MS;
		default:
			return 0;
	}
}

void execute_direction(Direction dir, int duration_ms) {
	char execBuf[64];
	sprintf(execBuf, "[底盘] 执行 %d 估算时间=%dms\n", dir, duration_ms);
	zx_uart_send_str((u8*)execBuf);

	if (any) {
		car_run(0, 0, 0, 0, 0);
		return;
	}

	// 使用默认时间（如果未指定）
	if (duration_ms <= 0) {
		duration_ms = get_default_time(dir);
	}

	switch (dir) {
		case DIR_STOP:           car_run(0, 0, 0, 0, 0); break;
		case DIR_FORWARD:        car_run(500, 500, 500, 500, duration_ms); break;
		case DIR_BACK:           car_run(-500, -500, -500, -500, duration_ms); break;
		case DIR_LEFT:           car_run(-500, 500, -500, 500, duration_ms); break;
		case DIR_RIGHT:          car_run(500, -500, 500, -500, duration_ms); break;
		case DIR_LEFT_FORWARD:   car_run(-500, 500, 500, -500, duration_ms); break;
		case DIR_RIGHT_FORWARD:  car_run(500, -500, -500, 500, duration_ms); break;
		case DIR_RIGCEN:         car_run(500, 0, 500, 0, duration_ms); break;
		case DIR_RIGCEN_REV:     car_run(-500, 0,-500, 0, duration_ms); break;
		case DIR_LEFCEN:         car_run(0, -500, 0, -500, duration_ms); break;
		case DIR_LEFCEN_REV:     car_run(0, 500, 0, 500, duration_ms); break;
		default:                 car_run(0, 0, 0, 0, 0); break;
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
    tb_delay_ms(5);

    ultra_front = get_csb_value(SENSOR_FRONT);
    tb_delay_ms(5);

    ultra_right = get_csb_value(SENSOR_RIGHT);
    tb_delay_ms(5);

    ultra_back = get_csb_value(SENSOR_BACK);
    tb_delay_ms(10);

    char buf[64];
    sprintf(buf, "[超声波] L=%d F=%d R=%d B=%d\n", ultra_left, ultra_front, ultra_right, ultra_back);
    zx_uart_send_str((u8*)buf);
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

void reset_avoid_state() {
    avoid_mode = FALSE;
    avoid_state = AVOID_IDLE;
    main_direction = DIR_STOP;
    avoid_distance_done = 0;
    avoid_target_distance = 0;
}

void plan_next_direction() {
    if (ultra_front > SAFE_THRESHOLD) {
        main_direction = DIR_FORWARD;
        avoid_target_distance = SAFE_THRESHOLD; // 每次前进20cm
        zx_uart_send_str((u8*)"[避障] 前方仍可前进一段，继续前进\n");
    } else {
        // 进入绕行
        if (ultra_left > SAFE_THRESHOLD) {
            main_direction = DIR_LEFT;
            avoid_target_distance = 30;
            zx_uart_send_str((u8*)"[避障] 前方受阻，尝试向左绕行\n");
        } else if (ultra_right > SAFE_THRESHOLD) {
            main_direction = DIR_RIGHT;
            avoid_target_distance = 30;
            zx_uart_send_str((u8*)"[避障] 前方受阻，尝试向右绕行\n");
        } else if (ultra_back > SAFE_THRESHOLD) {
            main_direction = DIR_BACK;
            avoid_target_distance = 30;
            zx_uart_send_str((u8*)"[避障] 无路可走，尝试后退避障\n");
        } else {
            main_direction = DIR_STOP;
            zx_uart_send_str((u8*)"[避障] 全方位阻挡，挂起等待\n");
        }
    }
}

void execute_current_step() {
    int exec_time = TIME_FROM_DIST(avoid_target_distance) / 1000;
    char dbg[64];
    sprintf(dbg, "[避障] 执行方向=%d 距离=%d 时间=%dms\n", main_direction, avoid_target_distance, exec_time);
    zx_uart_send_str((u8*)dbg);
    execute_direction(main_direction, exec_time);

    // 只有主方向前进才记录进度
    if (main_direction == DIR_FORWARD) {
        forward_done += avoid_target_distance;
    }
}

void check_if_goal_reached() {
    if (forward_done >= main_distance) {
        zx_uart_send_str((u8*)"[避障] 已到达目标主距离，退出避障\n");
        reset_avoid_state();
    } else {
        zx_uart_send_str((u8*)"[避障] 主路程未完成，继续导航\n");
        avoid_state = AVOID_PLANNING;
    }
}

void avoid_system() {
    executed_in_this_loop = FALSE;
    if (!avoid_mode) return;

    switch (avoid_state) {
        case AVOID_IDLE:
            zx_uart_send_str((u8*)"[避障] 初始规划\n");
            avoid_distance_done = 0;
            avoid_state = AVOID_PLANNING;
            break;

        case AVOID_PLANNING:
            plan_next_direction();
            avoid_state = AVOID_EXECUTING;
            break;

        case AVOID_EXECUTING:
            if (main_direction != DIR_STOP) {
                execute_current_step();
            }
            avoid_state = AVOID_CHECKING;
            break;

        case AVOID_CHECKING:
            check_if_goal_reached();
            break;

        default:
            reset_avoid_state();
            break;
    }

    executed_in_this_loop = TRUE;
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
	
	//tb_delay_ms(2000);
	//car_run(1000, 1000, 1000, 1000);
	//zx_uart_send_str("{#006P1000T2000!#007P1000T2000!#008P1000T2000!#009P1000T2000!}");

	while (1) {
		// 1 - 基础刷新
		loop_nled();	    
		ultra_distance();	
		tb_delay_ms(50);

		// 2 - 串口处理 + 避障
		loop_uart();	
		avoid_system();	
		tb_delay_ms(50); // 避障系统处理延时

		// 3 - 底盘动作（若未被避障系统控制）
		if (!executed_in_this_loop) {
			execute_direction(main_direction, -1);	
		}
		tb_delay_ms(200);
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
    if (!uart_frame.ready) return;

    // 拷贝数据并清空标志
    u8 mode = uart_frame.mode;
    u8 buf[UART_BUF_SIZE];
    memcpy(buf, uart_frame.buf, uart_frame.index + 1);
    
    uart_frame.ready = 0;
    uart_frame.index = 0;
    uart_frame.mode = 0;

    // 实际处理
    switch(mode) {
        case 1: // $
            beep_on_times(1, 100);
            parse_cmd(buf);
            break;
        case 5: // @
            beep_on_times(2, 100);
            avoid_mode = TRUE;
            dirs(buf);
            break;
        default:
            // 可扩展其他处理
            break;
    }
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
void car_run(int speedlq, int speedrq, int speedlh, int speedrh, int duration_ms) {
	sprintf((char *)cmd_return,
		"{#006P%04dT%04d!#007P%04dT%04d!#008P%04dT%04d!#009P%04dT%04d!}",
		(int)(1500 + speedlq), duration_ms,
		(int)(1500 - speedrq), duration_ms,
		(int)(1500 + speedlh), duration_ms,
		(int)(1500 - speedrh), duration_ms
	);
	zx_uart_send_str(cmd_return);
}
