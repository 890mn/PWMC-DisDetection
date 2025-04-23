#include "z_usart.h"

void tb_usart_init(void) {
	tb_usart1_init(115200);
	uart1_open();
	
	tb_usart2_init(115200);
	uart2_open();
	
	tb_usart3_init(115200);
	uart3_open();
	
	tb_interrupt_open();
	return;
}

void tb_usart1_init(u32 rate) {  
    GPIO_InitTypeDef GPIO_InitStructure;  
    USART_InitTypeDef USART_InitStructure; 
	USART_ClockInitTypeDef USART_ClockInitStructure; 	
    NVIC_InitTypeDef NVIC_InitStructure;  
  
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);  
    USART_DeInit(USART1);  
    /* Configure USART Tx as alternate function push-pull */  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
      
    /* Configure USART Rx as input floating */  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
  
    USART_InitStructure.USART_BaudRate = rate;  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
    
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;  
    USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;  
    USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;  
    USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;  
    USART_ClockInit(USART1, &USART_ClockInitStructure);  
	USART_Init(USART1, &USART_InitStructure );   
  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure); 
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	
//	USART_ITConfig(USART1, USART_IT_PE, ENABLE);
//	USART_ITConfig(USART1, USART_IT_ERR, ENABLE);

	USART_Cmd(USART1, ENABLE);  
}  
  
void tb_usart2_init(u32 rate) {  
	GPIO_InitTypeDef GPIO_InitStructure;  
    USART_InitTypeDef USART_InitStructure;   
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  
    USART_DeInit(USART2);  
    /* Configure USART Tx as alternate function push-pull */  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
      
    /* Configure USART Rx as input floating */  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
    USART_InitStructure.USART_BaudRate = rate;  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
	USART_Init(USART2, &USART_InitStructure );   
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);   
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
//	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
//	USART_ITConfig(USART2, USART_IT_PE, ENABLE);
//	USART_ITConfig(USART2, USART_IT_ERR, ENABLE);
	
	USART_Cmd(USART2, ENABLE); 
} 

void tb_usart3_init(u32 rate) {  
	GPIO_InitTypeDef GPIO_InitStructure;  
    USART_InitTypeDef USART_InitStructure;   
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	
	/* config USART3 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	/* USART3 GPIO config */
	/* Configure USART3 Tx (PB.10) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);    
	
	/* Configure USART3 Rx (PB.11) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* USART3 mode config */
	USART_InitStructure.USART_BaudRate = rate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
		
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);   
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  //USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
	
	//USART_HalfDuplexCmd(USART3,ENABLE);
		
	USART_Cmd(USART3, ENABLE); 
} 

//从串口1发送一个字节
void tb_usart1_send_byte(u8 Data) {
	USART_SendData(USART1, Data);
	return;
}

void tb_usart1_send_nbyte(u8 *Data, u16 size) {
	u16 i = 0;
	for(i=0; i<size; i++) {
		USART_SendData(USART1, Data[i]);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
	}
	return;
}
void tb_usart1_send_str(u8 *Data) {
	while(*Data) {
		USART_SendData(USART1, *Data++);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
	}
	return;
}

//从串口2发送一个字节
void tb_usart2_send_byte(u8 Data) {
	USART_SendData(USART2, Data);
	return;
}

void tb_usart2_send_nbyte(u8 *Data, u16 size) {
	u16 i = 0;
	for(i=0; i<size; i++) {
		USART_SendData(USART2, Data[i]);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); 
	}
	return;
}
void tb_usart2_send_str(u8 *Data) {
	while(*Data) {
		USART_SendData(USART2, *Data++);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); 
	}
	return;
}

//从串口3发送一个字节
void tb_usart3_send_byte(u8 Data) {
	USART_SendData(USART3, Data);
	return;
}

void tb_usart3_send_nbyte(u8 *Data, u16 size) {
	u16 i = 0;
	for(i=0; i<size; i++) {
		USART_SendData(USART3, Data[i]);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET); 
	}
	return;
}
void tb_usart3_send_str(u8 *Data) {
	while(*Data) {
		USART_SendData(USART3, *Data++);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET); 
	}
	return;
}


/**========================中断处理函数=============================**/
//串口收发中断处理函数
int USART1_IRQHandler(void) {
	u8 sbuf;

    if (USART_GetFlagStatus(USART1, USART_IT_RXNE) == SET) {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        sbuf = USART_ReceiveData(USART1);
        
        // 忽略新数据直到主循环处理完上一次
        if (uart_frame.ready) return 0;

        // 如果接收到起始符，重新开始收集
        if (sbuf == '$')      uart_frame.mode = 1;
        else if (sbuf == '#') uart_frame.mode = 2;
        else if (sbuf == '{') uart_frame.mode = 3;
        else if (sbuf == '<') uart_frame.mode = 4;
        else if (sbuf == '@') uart_frame.mode = 5;

        // 接收数据
        if (uart_frame.index < UART_BUF_SIZE) {
            uart_frame.buf[uart_frame.index++] = sbuf;
        }

        // 接收结束条件判断
        if ((uart_frame.mode == 1 || uart_frame.mode == 2 || uart_frame.mode == 5) && sbuf == '!') {
            uart_frame.buf[uart_frame.index] = '\0';
            uart_frame.ready = 1;
        } else if (uart_frame.mode == 3 && sbuf == '}') {
            uart_frame.buf[uart_frame.index] = '\0';
            uart_frame.ready = 1;
        } else if (uart_frame.mode == 4 && sbuf == '>') {
            uart_frame.buf[uart_frame.index] = '\0';
            uart_frame.ready = 1;
        }

        // 超限保护
        if (uart_frame.index >= UART_BUF_SIZE) {
            uart_frame.index = 0;
        }
    }

    return 0;
}

int USART3_IRQHandler(void) { 
	u8 sbuf;

    if (USART_GetFlagStatus(USART3, USART_IT_RXNE) == SET) {
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
        sbuf = USART_ReceiveData(USART3);
        
        // 忽略新数据直到主循环处理完上一次
        if (uart_frame.ready) return 0;

        // 如果接收到起始符，重新开始收集
        if (sbuf == '$')      uart_frame.mode = 1;
        else if (sbuf == '#') uart_frame.mode = 2;
        else if (sbuf == '{') uart_frame.mode = 3;
        else if (sbuf == '<') uart_frame.mode = 4;
        else if (sbuf == '@') uart_frame.mode = 5;

        // 接收数据
        if (uart_frame.index < UART_BUF_SIZE) {
            uart_frame.buf[uart_frame.index++] = sbuf;
        }

        // 接收结束条件判断
        if ((uart_frame.mode == 1 || uart_frame.mode == 2 || uart_frame.mode == 5) && sbuf == '!') {
            uart_frame.buf[uart_frame.index] = '\0';
            uart_frame.ready = 1;
        } else if (uart_frame.mode == 3 && sbuf == '}') {
            uart_frame.buf[uart_frame.index] = '\0';
            uart_frame.ready = 1;
        } else if (uart_frame.mode == 4 && sbuf == '>') {
            uart_frame.buf[uart_frame.index] = '\0';
            uart_frame.ready = 1;
        }

        // 超限保护
        if (uart_frame.index >= UART_BUF_SIZE) {
            uart_frame.index = 0;
        }
    }

    return 0;
}

void uart1_send_str(u8 *str) {
	tb_usart1_send_str(str);
}

void uart1_send_nbyte(u8 *Data, u16 size) {
	tb_usart1_send_nbyte(Data, size);
}

void uart1_send_byte(u8 data) {
	tb_usart1_send_byte(data);
}



void uart2_send_str(u8 *str) {
	tb_usart2_send_str(str);	
}


void uart2_send_nbyte(u8 *Data, u16 size) {
	tb_usart2_send_nbyte(Data, size);
}

void uart2_send_byte(u8 data) {
	tb_usart2_send_byte(data);
}


void zx_uart_send_str(u8 *str) {
	uart1_get_ok  = 1;
	tb_usart3_send_str(str);
	tb_usart1_send_str(str);		
	uart1_get_ok = 0;
}


void uart3_send_str(u8 *str) {
	tb_usart3_send_str(str);	
}


void uart3_send_nbyte(u8 *Data, u16 size) {
	tb_usart3_send_nbyte(Data, size);
}

void uart3_send_byte(u8 data) {
	tb_usart3_send_byte(data);
}
