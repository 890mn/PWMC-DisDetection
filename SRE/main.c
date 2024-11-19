#include "main.h"
#include "lcd.h"
#include "tim.h"
#include "nvic.h"
#include "usart.h"
#include "gpio.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define STEP_01_10 1     
#define MAX_X_01_10 200  

uint8_t echo_flag = 0;
uint8_t rx_index = 0;
uint8_t buf[32];
uint8_t rx_buffer[32];

uint16_t rising_cnt = 0;
uint16_t falling_cnt = 0;
uint16_t Start_DisX = 47;
uint16_t Start_DisY = 270;
uint16_t Start_DutX = 195;
uint16_t Start_DutY = 270;
uint16_t pwm_lookup_table[201];

// 全局变量
QueueHandle_t distanceQueue; // 用于传递距离
SemaphoreHandle_t lcdSemaphore = NULL; // 用于 LCD 显示资源保护

// 初始化 PWM 查找表
void Init_PWM_LookupTable(void) 
{
    for (uint16_t i = 0; i < 201; ++i) 
        pwm_lookup_table[i] = (849 / 43) * sqrt(1849 - (i * i) / (STEP_01_10 * STEP_01_10)) - 30; // Delta +- 30
}

void Motor_Control(float distance)
{
    float pwm_pulse;
	
    // Fitting curve implementation
    if(distance >= 0 && distance <= 20) pwm_pulse = pwm_lookup_table[(uint16_t)distance];
    else if(distance > 20 && distance <= 100) pwm_pulse = - 9.9 * distance + 948 - 60; //Delta +- 60
    else pwm_pulse = 0;
	
    // PWM Duty Cycle
    uint16_t pwm_value = (uint16_t)(pwm_pulse / 849.0f * 100.0f); 

    // Sey PWM Output
    TIM_SetCompare2(TIM2, (uint16_t)pwm_pulse);

    GPIO_SetBits(GPIOA, GPIO_Pin_1); // IN1 = 1

    sprintf((char *)buf, "PWM Duty Cycle: %.2d%%\r\n", pwm_value);
    USART_SendString((int8_t *)buf);

    LED_Control(LEDALL, 0);
    LED_PWM(pwm_value);
}

void Update_Octagons(float distance, float pwm_pulse)
{
    uint16_t dis_x = Start_DisX; // Left octagon (distance)
    uint16_t dis_y = Start_DisY;

    uint16_t dut_x = Start_DutX; // Right octagon (duty)
    uint16_t dut_y = Start_DutY;

    uint16_t bar_width = 30;
    uint16_t bar_max_height = 100;
    uint16_t block_height = 10; // Height of each block for segmented display

    // Filter the distance input
    float filtered_distance = Filter_Distance(distance);

    // Calculate the number of blocks
    uint8_t target_dis_blocks = (uint8_t)((filtered_distance / 40.0f) * (bar_max_height / block_height));
    if (target_dis_blocks > (bar_max_height / block_height))
        target_dis_blocks = bar_max_height / block_height; // Limit the height of the distance bar

    uint8_t target_duty_blocks = (uint8_t)(pwm_pulse * (bar_max_height / block_height));

    // Track the number of blocks from the previous update
    static uint8_t last_dis_blocks = 0;
    static uint8_t last_duty_blocks = 0;

    // Update the bar graph
    if (target_dis_blocks != last_dis_blocks)
        // Incrementally update the bar graph to ensure each block is drawn or cleared
        while (last_dis_blocks != target_dis_blocks)
            if (last_dis_blocks < target_dis_blocks) {
                // Draw new blocks
                LCD_FillRect(dis_x - bar_width / 2, dis_y - 60 - last_dis_blocks * block_height, bar_width, block_height, White);
                ++last_dis_blocks;
            }
            else if (last_dis_blocks > target_dis_blocks) {
                // Clear extra blocks
                --last_dis_blocks;
                LCD_FillRect(dis_x - bar_width / 2, dis_y - 60 - last_dis_blocks * block_height, bar_width, block_height, Black);
            }

    if (target_duty_blocks != last_duty_blocks)
        // Incrementally update the bar graph to ensure each block is drawn or cleared
        while (last_duty_blocks != target_duty_blocks)
            if (last_duty_blocks < target_duty_blocks) {
                // Draw new blocks
                LCD_FillRect(dut_x - bar_width / 2, dut_y - 60 - last_duty_blocks * block_height, bar_width, block_height, White);
                last_duty_blocks++;
            }
            else if (last_duty_blocks > target_duty_blocks) {
                // Clear extra blocks
                last_duty_blocks--;
                LCD_FillRect(dut_x - bar_width / 2, dut_y - 60 - last_duty_blocks * block_height, bar_width, block_height, Black);
            }

    // Draw the octagons surrounding the bars
    LCD_DrawOctagon(dis_x, dis_y, 22);
    LCD_DrawOctagon(dut_x, dut_y, 22);
}

// 超声波测距任务
void UltrasonicTask(void *pvParameters)
{
    float distance = 0;

    while (1) {
        // 触发超声波传感器
        GPIO_SetBits(GPIOA, GPIO_Pin_7); 
        vTaskDelay(pdMS_TO_TICKS(5));
        GPIO_ResetBits(GPIOA, GPIO_Pin_7);

        rising_cnt = 0;
        falling_cnt = 0;
        echo_flag = 0;
        TIM_SetCounter(TIM3, 0);

        TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
        TIM_Cmd(TIM3, ENABLE);

        for (uint16_t i = 0; i < 10; ++i) {
            if (rising_cnt && falling_cnt) {
                distance = (falling_cnt - rising_cnt) * 0.017f; // 计算距离
                xQueueSend(distanceQueue, &distance, 0); // 将距离发送到队列
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);
        TIM_Cmd(TIM3, DISABLE);

        vTaskDelay(pdMS_TO_TICKS(200)); // 每 200ms 测一次
    }
}

// 电机控制任务
void MotorControlTask(void *pvParameters)
{
    float distance = 0;

    while (1) {
        if (xQueueReceive(distanceQueue, &distance, portMAX_DELAY)) {
            Motor_Control(distance); // 控制电机
        }
    }
}

// LCD 更新任务
void LCDUpdateTask(void *pvParameters)
{
    float distance = 0;

    while (1) {
        if (xQueueReceive(distanceQueue, &distance, portMAX_DELAY)) {
            if (xSemaphoreTake(lcdSemaphore, portMAX_DELAY)) {
                Update_Octagons(distance, 1.0f - (distance / 40.0f)); // 更新显示
                xSemaphoreGive(lcdSemaphore);
            }
        }
    }
}

int main(void)
{
    // 硬件初始化
    SysTick_Config(SystemCoreClock / 1000);
    USART_Config();
    GPIO_Config();
    TIM3_Config();
    TIM2_Config();
    NVIC_Config();
    LCD_Back_Init();
    Init_PWM_LookupTable();

    // 创建 RTOS 资源
    distanceQueue = xQueueCreate(20, sizeof(float));
    lcdSemaphore = xSemaphoreCreateMutex();

    // 创建任务
    xTaskCreate(UltrasonicTask, "Ultrasonic", 256, NULL, 2, NULL);
    xTaskCreate(MotorControlTask, "MotorControl", 256, NULL, 2, NULL);
    xTaskCreate(LCDUpdateTask, "LCDUpdate", 256, NULL, 1, NULL);

    // 启动调度器
    vTaskStartScheduler();

    while (1) {
        // 如果运行到这里，说明调度器启动失败
    }
}
