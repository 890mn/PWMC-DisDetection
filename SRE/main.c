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
    // �����˲�������
    distance_buffer[filter_index] = new_distance;
    filter_index = (filter_index + 1) % FILTER_SIZE;

    // �����˲���ľ���
    float filtered_distance = 0;
    for (uint8_t i = 0; i < FILTER_SIZE; ++i)
        filtered_distance += distance_buffer[i];

    // �������仯�ϴ󣬼�С�˲�Ч����������Ӧ
    if (fabs(new_distance - filtered_distance / FILTER_SIZE) > 5.0f)
        // ����仯�ϴ�ʱ��ֱ��ʹ������ֵ
				return new_distance;  

    return filtered_distance / FILTER_SIZE;  // ����ʹ��ƽ���˲�
}




/**
 * @˵��     ������ƺ��������ݲ�õľ����������ٶ� (PWM ռ�ձ�)
 * @����     distance: �������ľ���
 * @����ֵ   None
 */
void Motor_Control(float distance)
{
    float pwm_pulse = 1.0f - (distance / 40.0f);
    uint16_t pwm_value = (uint16_t)(pwm_pulse * 909.0f); 

    // �������ռ�ձ���ֵ�����ڸ�ֵʱ�رյ��
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

    // ��ӡ PWM ռ�ձ�
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
    uint16_t dis_x = Start_DisX;    // ���˱��Σ�dis��
    uint16_t dis_y = Start_DisY;    

    uint16_t dut_x = Start_DutX;    // �Ҳ�˱��Σ�duty��
    uint16_t dut_y = Start_DutY;    

    uint16_t bar_width = 30;       
    uint16_t bar_max_height = 100;  
    uint16_t block_height = 10;     // ÿ����ĸ߶ȣ��ֿ���ʾ

    // �˲���ľ���
    float filtered_distance = Filter_Distance(distance);

    // ���� dis �������ķֿ�����
    uint8_t target_dis_blocks = (uint8_t)((filtered_distance / 40.0f) * (bar_max_height / block_height));
    if (target_dis_blocks > (bar_max_height / block_height)) 
        target_dis_blocks = bar_max_height / block_height; // ���� dis �ĸ߶�

    // ���� duty �������ķֿ�����
    uint8_t target_duty_blocks = (uint8_t)(pwm_pulse * (bar_max_height / block_height));

    // ��¼��һ�εĿ���
    static uint8_t last_dis_blocks = 0;
    static uint8_t last_duty_blocks = 0;

    // ���� dis ����������
    if (target_dis_blocks != last_dis_blocks)
    {
        // ʹ���𲽸��£�ȷ��ÿ���鶼����ȷ���ƻ����
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

    // ���� duty ����������
    if (target_duty_blocks != last_duty_blocks)
    {
        // ʹ���𲽸��£�ȷ��ÿ���鶼����ȷ���ƻ����
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

    // ���ư˱��Σ�ֻ����һ��
    LCD_DrawOctagon(dis_x, dis_y, 22);
    LCD_DrawOctagon(dut_x, dut_y, 22);
}

void LCD_Back_Init()
{
		STM3210B_LCD_Init();
    LCD_SetTextColor(White);
    LCD_SetBackColor(Black);
    LCD_Clear(Black);
	
		LCD_DrawOctagon(Start_DisX, Start_DisY, 22);  // ��ʼ dis �˱���
    LCD_DrawOctagon(Start_DutX, Start_DutY, 22);  // ��ʼ duty �˱���
    
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
 * @˵��     ��ʱ����
 * @����     nTime: ��ʱʱ��
 * @����ֵ   None
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
			
        // ����������������
        GPIO_SetBits(GPIOA, GPIO_Pin_7);   // PA7 - Trigger
        Delay_Ms(5);
        GPIO_ResetBits(GPIOA, GPIO_Pin_7);

        // ���ü������ͱ�־λ
        rising_cnt = 0;
        falling_cnt = 0;
        echo_flag = 0;
        TIM_SetCounter(TIM3, 0);

        // ���������ж�
        TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
        TIM_Cmd(TIM3, ENABLE);

        // �ȴ��������
        for (uint16_t i = 0; i < 10; ++i)
        {
            if (rising_cnt && falling_cnt)
            {
                float distance = (falling_cnt - rising_cnt) * 0.017;
							
								sprintf((char*)buf, "Distance: %.2f cm\r\n", distance);
								USART_SendString((int8_t *)buf);
                Motor_Control(distance);  // ���Ƶ��

                // ����ռ�ձȸ��°˱��κ;�����
                Update_Octagons(distance, 1.0f - (distance / 40.0f));
                break;
            }
            Delay_Ms(10);
        }

        // ֹͣ��ʱ�����ж�
        TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);
        TIM_Cmd(TIM3, DISABLE);

        Delay_Ms(200);
    }
}
