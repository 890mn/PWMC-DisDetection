#include "z_main.h"

uint16_t ultra_left = 0;
uint16_t ultra_front = 0;
uint16_t ultra_right = 0;
uint16_t ultra_back = 0;

uint16_t main_dis = 0;
Direction main_dir = DIR_STOP;

uint16_t cmd_dis = 0;
Direction cmd_dir = DIR_STOP;

uint16_t saved_dis = 0;
Direction saved_dir = DIR_STOP;

bool any = FALSE;
volatile bool executed_in_this_loop = FALSE;

typedef struct {
	bool mode;
	AvoidState state;
	Direction dir;
	int dis;
}Avoid_t;
Avoid_t Core = {FALSE, AST_IDLE, DIR_STOP, 0};

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

int get_default_time() {
	switch (main_dir) {
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

void execute_direction() {
	int duration_ms;
	if (any) {
		car_run(0, 0, 0, 0, 0);
		return;
	}

	if (Core.mode == TRUE) {
		duration_ms = DIST_TIME_X1000(main_dis) / 1000;
	} else {
		duration_ms = get_default_time();
	}

	char execBuf[64];
	sprintf(execBuf, "[底盘] 执行 %d 估算时间=%dms\n", main_dir, duration_ms);
	zx_uart_send_str((u8*)execBuf);

	switch (main_dir) {
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
		case DIR_STOP:
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
    cmd_dir = get_direction_from_str(directionStr);
    cmd_dis = atoi(numberStr);
}

void Core_reset() {
    Core.mode = FALSE;
    Core.dir = DIR_STOP;
	Core.dis = 0;
	main_dir = DIR_STOP;
}

void AST_Core() {
    if (Core.mode == FALSE) return;

    switch (Core.state) {
        case AST_IDLE:
            zx_uart_send_str((u8*)"[ASTCore] Core Start\n");
            Core.state = AST_CHECK;
            break;

        case AST_CHECK:
            if (Core.dis >= cmd_dis) {
                zx_uart_send_str((u8*)"[ASTCore] Core finish\n");
				Core_reset();
                Core.state = AST_IDLE;
                break;
            }

            if (ultra_front < SAFE_THRES_CM || ultra_back < SAFE_THRES_CM ||
                ultra_left < SAFE_THRES_CM || ultra_right < SAFE_THRES_CM) {
                
                zx_uart_send_str((u8*)"[ASTCore] ALERT PLAN\n");

                // 保存当前指令
                saved_dir = Core.dir;
                saved_dis = Core.dis;

                // 停车
                main_dir = DIR_STOP;

                // 切换到规划状态
                Core.state = AST_PLAN;
                break;
            }

            Core.dis += 10;
            main_dir = Core.dir;
            main_dis = Core.dis;
            char dbg[64];
            sprintf(dbg, "[ASTCore] MAIN_DIR=%d MAIN_DIS=%d\n", main_dir, main_dis);
            zx_uart_send_str((u8*)dbg);
            break;

        case AST_PLAN: {
            // 在PLAN模式下，尝试自动寻找绕行路径
            static int plan_step = 0; // 增加一个小状态机步进

            switch (plan_step) {
                case 0: // 第一步，判断左侧能否绕行
                    if (ultra_left > SAFE_THRES_CM) {
                        main_dir = DIR_LEFT;
                        zx_uart_send_str((u8*)"[ASTCore] PLAN:LEFT\n");
                        plan_step = 1;
                    } else if (ultra_right > SAFE_THRES_CM) {
                        main_dir = DIR_RIGHT;
                        zx_uart_send_str((u8*)"[ASTCore] PLAN:RIGHT\n");
                        plan_step = 1;
                    } else if (ultra_back > SAFE_THRES_CM) {
                        main_dir = DIR_BACK;
                        zx_uart_send_str((u8*)"[ASTCore] PLAN:BACK\n");
                        plan_step = 1;
                    } else {
                        main_dir = DIR_STOP;
                        zx_uart_send_str((u8*)"[ASTCore] PLAN:ALL BLOCK WAIT\n");
                        plan_step = 0;
                    }
                    break;

                case 1: // 第二步，执行避障动作一段时间
                    Core.dis += 10; // 避障过程也推进距离，模拟行驶
                    main_dis = Core.dis;
                    zx_uart_send_str((u8*)"[ASTCore] IN PLAN\n");

                    // 如果前方已经畅通，恢复原路线
                    if (ultra_front > SAFE_THRES_CM) {
                        Core.dir = saved_dir;
                        Core.dis = saved_dis;
                        zx_uart_send_str((u8*)"[ASTCore] PLAN COMPLETE\n");
                        plan_step = 0;
                        Core.state = AST_CHECK;
                    }
                    break;
            }
            break;
        }
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
		AST_Core();	
		tb_delay_ms(50); // 避障系统处理延时

		// 3 - 底盘动作
		execute_direction();	
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
            Core.mode = TRUE;
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
		main_dir = DIR_STOP;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$FORW!"), pos){
		main_dir = DIR_FORWARD;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$BACK!"), pos){
		main_dir = DIR_BACK;		
	}else if(pos = str_contain_str(cmd, (u8 *)"$LEFT!"), pos){
		main_dir = DIR_LEFT;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$RIGH!"), pos){
		main_dir = DIR_RIGHT;		
	}else if(pos = str_contain_str(cmd, (u8 *)"$LEFR!"), pos){
		main_dir = DIR_LEFT_FORWARD;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$LEBA!"), pos){
		main_dir = DIR_RIGHT_FORWARD;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$RICE!"), pos){
		main_dir = DIR_RIGCEN;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$RICR!"), pos){
		main_dir = DIR_RIGCEN_REV;	
	}else if(pos = str_contain_str(cmd, (u8 *)"$LECE!"), pos){
		main_dir = DIR_LEFCEN;
	}else if(pos = str_contain_str(cmd, (u8 *)"$LECR!"), pos){
		main_dir = DIR_LEFCEN_REV;
	}else if(pos = str_contain_str(cmd, (u8 *)"$STOPANY!"), pos){
		main_dir = DIR_STOP;
		any = TRUE;
		Core.mode = FALSE;
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
