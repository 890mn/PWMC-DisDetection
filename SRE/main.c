/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f10x.h"
#include "stdio.h"

#include "math.h"
#include "stdlib.h"
#include "string.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FILTER_SIZE 3
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float distance_buffer[FILTER_SIZE] = {0};

uint8_t filter_index = 0;
uint8_t echo_flag = 0;
uint8_t buf[32];
uint8_t rx_buffer[32];
uint8_t rx_index = 0;

uint16_t rising_cnt = 0;
uint16_t falling_cnt = 0;
uint16_t Start_DisX = 47;
uint16_t Start_DisY = 270;
uint16_t Start_DutX = 195;
uint16_t Start_DutY = 270;
extern uint16_t martix[][24];

uint32_t TimingDelay = 0;

float Filter_Distance(float new_distance)
{
    // 更新滤波缓冲区
    distance_buffer[filter_index] = new_distance;
    filter_index = (filter_index + 1) % FILTER_SIZE;

    // 计算滤波后的距离
    float filtered_distance = 0;
    for (uint8_t i = 0; i < FILTER_SIZE; ++i)
        filtered_distance += distance_buffer[i];

    // 如果距离变化较大，减小滤波效果，快速响应
    if (fabs(new_distance - filtered_distance / FILTER_SIZE) > 5.0f)
        // 距离变化较大时，直接使用最新值
				return new_distance;  

    return filtered_distance / FILTER_SIZE;  // 否则使用平均滤波
}




/**
 * @说明     电机控制函数，根据测得的距离调整电机速度 (PWM 占空比)
 * @参数     distance: 测量到的距离
 * @返回值   None
 */
void Motor_Control(float distance)
{
    float pwm_pulse = 1.0f - (distance / 40.0f);
    uint16_t pwm_value = (uint16_t)(pwm_pulse * 909.0f); 

    // 设置最低占空比阈值，低于该值时关闭电机
    uint16_t pwm_min_threshold = 60;

    if (pwm_value > pwm_min_threshold) {
        GPIO_SetBits(GPIOA, GPIO_Pin_1);   // IN1 = 1
    }
    else {
        GPIO_ResetBits(GPIOA, GPIO_Pin_1); // IN1 = 0
        pwm_value = 0;
        pwm_pulse = 0;
    }

    TIM_SetCompare2(TIM2, pwm_value);

    // 打印 PWM 占空比
    sprintf((char*)buf, "PWM Duty Cycle: %.2f%%\r\n", pwm_pulse * 100.0f);
    USART_SendString((int8_t *)buf);
		
    LED_Control(LEDALL, 0);
    LED_PWM(pwm_pulse);
}


void LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    uint16_t i, j;
    LCD_SetTextColor(color); 

    for (i = 0; i < height; ++i) 
        for (j = 0; j < width; ++j)
            LCD_DrawPixel(x + j, y + i, color); 
}

void Update_Octagons(float distance, float pwm_pulse)
{
    uint16_t dis_x = Start_DisX;    // 左侧八边形（dis）
    uint16_t dis_y = Start_DisY;    

    uint16_t dut_x = Start_DutX;    // 右侧八边形（duty）
    uint16_t dut_y = Start_DutY;    

    uint16_t bar_width = 30;       
    uint16_t bar_max_height = 100;  
    uint16_t block_height = 10;     // 每个块的高度，分块显示

    // 滤波后的距离
    float filtered_distance = Filter_Distance(distance);

    // 计算 dis 矩形条的分块数量
    uint8_t target_dis_blocks = (uint8_t)((filtered_distance / 40.0f) * (bar_max_height / block_height));
    if (target_dis_blocks > (bar_max_height / block_height)) 
        target_dis_blocks = bar_max_height / block_height; // 限制 dis 的高度

    // 计算 duty 矩形条的分块数量
    uint8_t target_duty_blocks = (uint8_t)(pwm_pulse * (bar_max_height / block_height));

    // 记录上一次的块数
    static uint8_t last_dis_blocks = 0;
    static uint8_t last_duty_blocks = 0;

    // 更新 dis 矩形条部分
    if (target_dis_blocks != last_dis_blocks)
    {
        // 使用逐步更新，确保每个块都被正确绘制或清除
        while (last_dis_blocks != target_dis_blocks)
        {
            if (last_dis_blocks < target_dis_blocks) {
                LCD_FillRect(dis_x - bar_width / 2, dis_y - 60 - last_dis_blocks * block_height, bar_width, block_height, White);
                last_dis_blocks++;
            } else if (last_dis_blocks > target_dis_blocks) {
                last_dis_blocks--;
                LCD_FillRect(dis_x - bar_width / 2, dis_y - 60 - last_dis_blocks * block_height, bar_width, block_height, Black);
            }
        }
    }

    // 更新 duty 矩形条部分
    if (target_duty_blocks != last_duty_blocks)
    {
        // 使用逐步更新，确保每个块都被正确绘制或清除
        while (last_duty_blocks != target_duty_blocks)
        {
            if (last_duty_blocks < target_duty_blocks) {
                LCD_FillRect(dut_x - bar_width / 2, dut_y - 60 - last_duty_blocks * block_height, bar_width, block_height, White);
                last_duty_blocks++;
            } else if (last_duty_blocks > target_duty_blocks) {
                last_duty_blocks--;
                LCD_FillRect(dut_x - bar_width / 2, dut_y - 60 - last_duty_blocks * block_height, bar_width, block_height, Black);
            }
        }
    }

    // 绘制八边形，只绘制一次
    LCD_DrawOctagon(dis_x, dis_y, 22);
    LCD_DrawOctagon(dut_x, dut_y, 22);
}

void LCD_Back_Init()
{
		STM3210B_LCD_Init();
    LCD_SetTextColor(White);
    LCD_SetBackColor(Black);
    LCD_Clear(Black);
	
		LCD_DrawOctagon(Start_DisX, Start_DisY, 22);  // 初始 dis 八边形
    LCD_DrawOctagon(Start_DutX, Start_DutY, 22);  // 初始 duty 八边形
    
		liftDisplay(20, 40, martix['D'-32]);
		liftDisplay(30, 40, martix['I'-32]);
		liftDisplay(40, 40, martix['S'-32]);
		liftDisplay(30, 60, martix['T'-32]);
		liftDisplay(40, 60, martix['A'-32]);
		liftDisplay(50, 60, martix['N'-32]);
		liftDisplay(60, 60, martix['C'-32]);
		liftDisplay(70, 60, martix['E'-32]);
		
		liftDisplay(160, 40, martix['D'-32]);
		liftDisplay(170, 40, martix['U'-32]);
		liftDisplay(180, 40, martix['T'-32]);
		liftDisplay(190, 40, martix['Y'-32]);
		liftDisplay(170, 60, martix['C'-32]);
		liftDisplay(180, 60, martix['Y'-32]);
		liftDisplay(190, 60, martix['C'-32]);
		liftDisplay(200, 60, martix['L'-32]);
		liftDisplay(210, 60, martix['E'-32]);
}



/**
 * @说明     延时函数
 * @参数     nTime: 延时时间
 * @返回值   None
 */
void Delay_Ms(uint32_t nTime)
{
    TimingDelay = nTime;
    while (TimingDelay != 0) ;
}


int main(void)
{
    SysTick_Config(SystemCoreClock / 1000);

    USART_Config();
    GPIO_Config();
    TIM3_Config();
    TIM2_Config();
    NVIC_Config();

    LED_Control(LEDALL, 0);
		LCD_Back_Init();
		
    while (1) {
			
        // 触发超声波传感器
        GPIO_SetBits(GPIOA, GPIO_Pin_7);   // PA7 - Trigger
        Delay_Ms(5);
        GPIO_ResetBits(GPIOA, GPIO_Pin_7);

        // 重置计数器和标志位
        rising_cnt = 0;
        falling_cnt = 0;
        echo_flag = 0;
        TIM_SetCounter(TIM3, 0);

        // 启动捕获中断
        TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
        TIM_Cmd(TIM3, ENABLE);

        // 等待捕获完成
        for (uint16_t i = 0; i < 10; ++i)
        {
            if (rising_cnt && falling_cnt)
            {
                float distance = (falling_cnt - rising_cnt) * 0.017;
							
								sprintf((char*)buf, "Distance: %.2f cm\r\n", distance);
								USART_SendString((int8_t *)buf);
                Motor_Control(distance);  // 控制电机

                // 根据占空比更新八边形和矩形条
                Update_Octagons(distance, 1.0f - (distance / 40.0f));
                break;
            }
            Delay_Ms(10);
        }

        // 停止定时器和中断
        TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);
        TIM_Cmd(TIM3, DISABLE);

        Delay_Ms(200);
    }
}
