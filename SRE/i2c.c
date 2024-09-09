#include "stm32f10x.h"

#define I2C_PORT GPIOB
#define SDA_Pin	GPIO_Pin_7
#define SCL_Pin GPIO_Pin_6

#define FAILURE 0
#define SUCCESS 1

void SDA_Input_Mode()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = SDA_Pin;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	 

  	GPIO_Init(I2C_PORT, &GPIO_InitStructure);
}

void SDA_Output_Mode()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = SDA_Pin;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

  	GPIO_Init(I2C_PORT, &GPIO_InitStructure);
}

void SDA_Output( uint16_t val )
{
	if ( val ) {
		GPIO_SetBits(I2C_PORT,SDA_Pin);
	} else {
		GPIO_ResetBits(I2C_PORT,SDA_Pin);
	}
}

void SCL_Output( uint16_t val )
{
	if ( val ) {
		GPIO_SetBits(I2C_PORT,SCL_Pin);
	} else {
		GPIO_ResetBits(I2C_PORT,SCL_Pin);
	}
}

uint8_t SDA_Input()
{
	return GPIO_ReadInputDataBit( I2C_PORT, SDA_Pin);
}

void delay1(unsigned int n)
{
	unsigned int i;
	for ( i=0;i<n;++i);
}

void I2CStart(void)
{
	SDA_Output(1);delay1(500);
	SCL_Output(1);delay1(500);
	SDA_Output(0);delay1(500);
	SCL_Output(0);delay1(500);
}

void I2CStop(void)
{
	SCL_Output(0); delay1(500);
	SDA_Output(0); delay1(500);
	SCL_Output(1); delay1(500);
	SDA_Output(1); delay1(500);

}

unsigned char I2CWaitAck(void)
{
	unsigned short cErrTime = 5;
	SDA_Input_Mode(); 
	delay1(500);
	SCL_Output(1);delay1(500);
	while(SDA_Input())
	{
		cErrTime--;
		delay1(500);
		if (0 == cErrTime)
		{
			SDA_Output_Mode();
			I2CStop();
			return FAILURE;
		}
	}
	SDA_Output_Mode();
	SCL_Output(0);delay1(500); 
	return SUCCESS;
}

void I2CSendAck(void)
{
	SDA_Output(0);delay1(500);
	delay1(500);
	SCL_Output(1); delay1(500);
	SCL_Output(0); delay1(500);

}

void I2CSendNotAck(void)
{
	SDA_Output(1);
	delay1(500);
	SCL_Output(1); delay1(500);
	SCL_Output(0); delay1(500);

}

void I2CSendByte(unsigned char cSendByte)
{
	unsigned char  i = 8;
	while (i--)
	{
		SCL_Output(0);delay1(500); 
		SDA_Output(cSendByte & 0x80); delay1(500);
		cSendByte += cSendByte;
		delay1(500); 
		SCL_Output(1);delay1(500); 
	}
	SCL_Output(0);delay1(500); 
}

unsigned char I2CReceiveByte(void)
{
	unsigned char i = 8;
	unsigned char cR_Byte = 0;
	SDA_Input_Mode(); 
	while (i--)
	{
		cR_Byte += cR_Byte;
		SCL_Output(0);delay1(500); 
		delay1(500); 
		SCL_Output(1);delay1(500); 
		cR_Byte |=  SDA_Input(); 
	}
	SCL_Output(0);delay1(500); 
	SDA_Output_Mode();
	return cR_Byte;
}

void i2c_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = SDA_Pin | SCL_Pin;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

  	GPIO_Init(I2C_PORT, &GPIO_InitStructure);

}
