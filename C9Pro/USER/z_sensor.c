#include "z_main.h"

u16 kms_y = 0;
u8 get_count = 0;
u8 carry_step = 0;

/*
	辅助控 S2  TRIG - PA0 - 白线  Echo - PA1 - 绿线
	传感器 S3  TRIG - PB0 - 白线  Echo - PA2 - 绿线
	传感器 S4  TRIG - PB1 - 白线  Echo - PA3 - 绿线
	传感器 S5  TRIG - PA6 - 白线  Echo - PA4 - 绿线
	传感器 S6  TRIG - PA7 - 白线  Echo - PA5 - 绿线
		
*/
uint16_t trig_pins[] = {GPIO_Pin_0, GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_6, GPIO_Pin_7};
uint16_t echo_pins[] = {GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3, GPIO_Pin_4, GPIO_Pin_5};
GPIO_TypeDef *trig_ports[] = {GPIOA, GPIOB, GPIOB, GPIOA, GPIOA};
GPIO_TypeDef *echo_ports[] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOA};

//初始化传感器IO口
void setup_sensor(void) {
	setup_xunji();
	setup_csb();
}

void setup_csb(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);  
	
	for (int i = 0; i < 5; i++)
	{
		// 配置Trig引脚
		GPIO_InitStructure.GPIO_Pin = trig_pins[i];
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(trig_ports[i], &GPIO_InitStructure);

		// 配置Echo引脚
		GPIO_InitStructure.GPIO_Pin = echo_pins[i];
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(echo_ports[i], &GPIO_InitStructure);
	}
	
	//初始化超声波定时器
	TIM3_Int_Init(30000, 71);
}

 // 循迹左 A0  A1  循迹右 B0 A2
void setup_xunji(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   
	GPIO_Init(GPIOA, &GPIO_InitStructure); 	
}

void csb_Delay_Us(uint16_t time)  //延时函数
{ 
	uint16_t i,j;
	for(i=0;i<time;i++)
  		for(j=0;j<9;j++);
}

/*************************************************************
函数名称：get_csb_value()
功能介绍：采集超声波数据
函数参数：无
返回值：  采集的数据  
*************************************************************/
int get_csb_value(uint8_t sensor_id) {
    uint16_t echo_time = 0;
    long timeout = 0;
    int distance = 0;

    // 发出 TRIG 脉冲
    GPIO_SetBits(trig_ports[sensor_id], trig_pins[sensor_id]);
    csb_Delay_Us(10);
    GPIO_ResetBits(trig_ports[sensor_id], trig_pins[sensor_id]);

    // 等待 ECHO 拉高（超时保护）
    timeout = 0;
    while (GPIO_ReadInputDataBit(echo_ports[sensor_id], echo_pins[sensor_id]) == 0) {
        if (++timeout > 30000) goto send_result; // 超时，跳转发送0
    }

    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);

    // 等待 ECHO 拉低（结束测量）
    timeout = 0;
    while (GPIO_ReadInputDataBit(echo_ports[sensor_id], echo_pins[sensor_id]) == 1) {
        if (++timeout > 60000) {
            TIM_Cmd(TIM3, DISABLE);
            goto send_result; // 超时，跳转发送0
        }
    }

    TIM_Cmd(TIM3, DISABLE);
    echo_time = TIM_GetCounter(TIM3);

    // 计算距离（cm）
    distance = (int)(echo_time * 0.017f + 0.5f);

    // 合理值范围检测
    if (distance <= 0 || distance > 600) {
        distance = 0;
    }

send_result:
    // 必发串口信息
    sprintf((char *)cmd_return, "[%d]UTime=%d", sensor_id, distance);
    uart3_send_str(cmd_return);

    return distance;
}
