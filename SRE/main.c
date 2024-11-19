#include "main.h"
#include "lcd.h"
#include "tim.h"
#include "nvic.h"
#include "usart.h"
#include "gpio.h"
#include "led.h"

#define STEP_01_10 1     // STEP = 0.1 -> 1
#define MAX_X_01_10 200  // MAX_X = 20 -> 200

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

SemaphoreHandle_t xEchoSemaphore;

void Init_PWM_LookupTable(void) 
{
    for (uint16_t i = 0; i < 201; ++i) 
        pwm_lookup_table[i] = (849 / 43) * sqrt(1849 - (i * i) / (STEP_01_10 * STEP_01_10)) - 20; // Delta +- 30
    
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
    LED_PWM(pwm_value - 48);
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
    if (target_dis_blocks != last_dis_blocks) {
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
		}
		
    if (target_duty_blocks != last_duty_blocks) {
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
		}
    
		// Draw the octagons surrounding the bars
    LCD_DrawOctagon(dis_x, dis_y, 22);
    LCD_DrawOctagon(dut_x, dut_y, 22);
}

int main(void)
{
    // Initialize hardware peripherals
    Hardware_Init();

    // Create a FreeRTOS task
    xTaskCreate(UltrasonicTask, "Ultrasonic Task", 256, NULL, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    while (1);
}

void InitSemaphore(void) {
    xEchoSemaphore = xSemaphoreCreateBinary();
    if (xEchoSemaphore == NULL) {
        // Handle semaphore creation failure
        while (1);
    }
}

void Hardware_Init(void) {
    SysTick_Config(SystemCoreClock / 1000);

    USART_Config();
    GPIO_Config();
    TIM3_Config();
    TIM2_Config();
    NVIC_Config();
		InitSemaphore();
    LCD_Back_Init();
    Init_PWM_LookupTable();
}

// Ultrasonic sensor task
void UltrasonicTask(void *pvParameters) {
    uint8_t buf[50];

    while (1) {
        // Trigger the Ultrasonic Sensor
        GPIO_SetBits(GPIOA, GPIO_Pin_7); // PA7 - Trigger
        vTaskDelay(pdMS_TO_TICKS(5));
        GPIO_ResetBits(GPIOA, GPIO_Pin_7);

        // Reset the status
        rising_cnt = 0;
        falling_cnt = 0;
        echo_flag = 0;
        TIM_SetCounter(TIM3, 0);

        // Start the TIM IT
        TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
        TIM_Cmd(TIM3, ENABLE);

        for (uint16_t i = 0; i < 10; ++i) {
            // Wait for interrupt status
            if (rising_cnt && falling_cnt) {
                // Calculate distance
                float distance = (falling_cnt - rising_cnt) * 0.017;

                sprintf((char *)buf, "Distance: %.2f cm\r\n", distance);
                USART_SendString((int8_t *)buf);

                // Control motor and update display
                Motor_Control(distance);
                Update_Octagons(distance, 1.0f - (distance / 50.0f));
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        // Stop the TIM IT
        TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);
        TIM_Cmd(TIM3, DISABLE);

        // Delay before next measurement
        //vTaskDelay(pdMS_TO_TICKS(185));
    }
}
