#include "lcd.h"
#include "math.h"
#include "stdlib.h"

#define CHAR_WIDTH  16
#define CHAR_HEIGHT 24
#define FILTER_SIZE 3

float distance_buffer[FILTER_SIZE] = {0};

uint8_t filter_index = 0;

uint16_t output[CHAR_WIDTH / 2][2];
extern uint16_t FontMartix[][24];

static vu16 TextColor = 0x0000, BackColor = 0xFFFF;
vu16 dummy;

extern void Delay_Ms(uint32_t nTime);

/*******************************************************************************
* Function Name  : Delay_LCD
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length.
* Output         : None
* Return         : None
*******************************************************************************/
void Delay_LCD(u16 n)
{
	u16 i,j;
	for (i = 0;i<n;++i)
		for(j=0;j<3000;++j);
}

/*
	uC8230型液晶控制器寄存器配置
*/
void REG_8230_Init(void)
{
	LCD_WriteReg(0x0000,0x0001);
	Delay_LCD(1000); 
	LCD_WriteReg(0x0001,0x0000);
	LCD_WriteReg(0x0010,0x1790);
	LCD_WriteReg(0x0060,0x2700);
	LCD_WriteReg(0x0061,0x0001);
	LCD_WriteReg(0x0046,0x0002);
	LCD_WriteReg(0x0013,0x8010);
	LCD_WriteReg(0x0012,0x80fe);
	LCD_WriteReg(0x0002,0x0500);
	LCD_WriteReg(0x0003,0x1030);
	
	LCD_WriteReg(0x0030,0x0303);
	LCD_WriteReg(0x0031,0x0303);
	LCD_WriteReg(0x0032,0x0303);
	LCD_WriteReg(0x0033,0x0300);
	LCD_WriteReg(0x0034,0x0003);
	LCD_WriteReg(0x0035,0x0303);
	LCD_WriteReg(0x0036,0x0014);
	LCD_WriteReg(0x0037,0x0303);
	LCD_WriteReg(0x0038,0x0303);
	LCD_WriteReg(0x0039,0x0303);
	LCD_WriteReg(0x003a,0x0300);
	LCD_WriteReg(0x003b,0x0003);
	LCD_WriteReg(0x003c,0x0303);
	LCD_WriteReg(0x003d,0x1400);
	  
	LCD_WriteReg(0x0092,0x0200);
	LCD_WriteReg(0x0093,0x0303);
	LCD_WriteReg(0x0090,0x080d); 
	LCD_WriteReg(0x0003,0x1018); 
	LCD_WriteReg(0x0007,0x0173);
}

void REG_932X_Init(void)
{
	LCD_WriteReg(R227, 0x3008);   // Set internal timing
	LCD_WriteReg(R231, 0x0012); // Set internal timing
	LCD_WriteReg(R239, 0x1231);   // Set internal timing
	LCD_WriteReg(R1  , 0x0000); // set SS and SM bit		  //0x0100
	LCD_WriteReg(R2  , 0x0700); // set 1 line inversion
	LCD_WriteReg(R3  , 0x1030);   // set GRAM write direction and BGR=1.
	LCD_WriteReg(R4  , 0x0000);   // Resize register
	LCD_WriteReg(R8  , 0x0207);   // set the back porch and front porch
	LCD_WriteReg(R9  , 0x0000);   // set non-display area refresh cycle ISC[3:0]
	LCD_WriteReg(R10 , 0x0000);   // FMARK function
	LCD_WriteReg(R12 , 0x0000); // RGB interface setting
	LCD_WriteReg(R13 , 0x0000);   // Frame marker Position
	LCD_WriteReg(R15 , 0x0000); // RGB interface polarity
	/**************Power On sequence ****************/
	LCD_WriteReg(R16 , 0x0000);   // SAP, BT[3:0], AP, DSTB, SLP, STB
	LCD_WriteReg(R17 , 0x0007);   // DC1[2:0], DC0[2:0], VC[2:0]
	LCD_WriteReg(R18 , 0x0000); // VREG1OUT voltage
	LCD_WriteReg(R19 , 0x0000);   // VDV[4:0] for VCOM amplitude
	Delay_Ms(200);                    // Delay 200 MS , Dis-charge capacitor power voltage
	LCD_WriteReg(R16 , 0x1690);   // SAP, BT[3:0], AP, DSTB, SLP, STB
	LCD_WriteReg(R17 , 0x0227); // R11H=0x0221 at VCI=3.3V, DC1[2:0], DC0[2:0], VC[2:0]
	Delay_Ms(50);      // Delay 50ms
	LCD_WriteReg(R18 , 0x001D); // External reference voltage= Vci;
	Delay_Ms(50);      // Delay 50ms
	LCD_WriteReg(R19 , 0x0800); // R13H=1D00 when R12H=009D;VDV[4:0] for VCOM amplitude
	LCD_WriteReg(R41 , 0x0014); // R29H=0013 when R12H=009D;VCM[5:0] for VCOMH
	LCD_WriteReg(R43 , 0x000B);   // Frame Rate = 96Hz
	Delay_Ms(50);      // Delay 50ms
	LCD_WriteReg(R32 , 0x0000); // GRAM horizontal Address
	LCD_WriteReg(R33 , 0x0000); // GRAM Vertical Address
	/* ----------- Adjust the Gamma Curve ---------- */
	LCD_WriteReg(R48 , 0x0007);
	LCD_WriteReg(R49 , 0x0707);
	LCD_WriteReg(R50 , 0x0006);
	LCD_WriteReg(R53 , 0x0704);
	LCD_WriteReg(R54 , 0x1F04);
	LCD_WriteReg(R55 , 0x0004);
	LCD_WriteReg(R56 , 0x0000);
	LCD_WriteReg(R57 , 0x0706);
	LCD_WriteReg(R60 , 0x0701);
	LCD_WriteReg(R61 , 0x000F);
	/* ------------------ Set GRAM area --------------- */
	LCD_WriteReg(R80 , 0x0000);   // Horizontal GRAM Start Address
	LCD_WriteReg(R81 , 0x00EF);   // Horizontal GRAM End Address
	LCD_WriteReg(R82 , 0x0000); // Vertical GRAM Start Address
	LCD_WriteReg(R83 , 0x013F); // Vertical GRAM Start Address
	LCD_WriteReg(R96 , 0x2700); // Gate Scan Line		  0xA700
	LCD_WriteReg(R97 , 0x0001); // NDL,VLE, REV
	LCD_WriteReg(R106, 0x0000); // set scrolling line
	/* -------------- Partial Display Control --------- */
	LCD_WriteReg(R128, 0x0000);   
	LCD_WriteReg(R129, 0x0000);
	LCD_WriteReg(R130, 0x0000);
	LCD_WriteReg(R131, 0x0000);
	LCD_WriteReg(R132, 0x0000);
	LCD_WriteReg(R133, 0x0000);
	/* -------------- Panel Control ------------------- */
	LCD_WriteReg(R144, 0x0010);
	LCD_WriteReg(R146, 0x0000);
	LCD_WriteReg(R147, 0x0003);
	LCD_WriteReg(R149, 0x0110);
	LCD_WriteReg(R151, 0x0000);
	LCD_WriteReg(R152, 0x0000);
	   /* Set GRAM write direction and BGR = 1 */
	   /* I/D=01 (Horizontal : increment, Vertical : decrement) */
	   /* AM=1 (address is updated in vertical writing direction) */
	LCD_WriteReg(R3  , 0x01018);  //0x1018
	
	LCD_WriteReg(R7  , 0x0173); // 262K color and display ON
}
/*******************************************************************************
* Function Name  : STM3210B_LCD_Init
* Description    : Initializes the LCD.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void STM3210B_LCD_Init(void)
{ 
	LCD_CtrlLinesConfig();
	dummy = LCD_ReadReg(0);	
	
	if(dummy == 0x8230){
		REG_8230_Init();
	}
	else{
		REG_932X_Init();	
	}
	dummy = LCD_ReadReg(0);	

}
/*******************************************************************************
* Function Name  : LCD_SetTextColor
* Description    : Sets the Text color.
* Input          : - Color: specifies the Text color code RGB(5-6-5).
* Output         : - TextColor: Text color global variable used by LCD_DrawChar
*                  and LCD_DrawPicture functions.
* Return         : None
*******************************************************************************/
void LCD_SetTextColor(vu16 Color)
{
	TextColor = Color;
}
/*******************************************************************************
* Function Name  : LCD_SetBackColor
* Description    : Sets the Background color.
* Input          : - Color: specifies the Background color code RGB(5-6-5).
* Output         : - BackColor: Background color global variable used by 
*                  LCD_DrawChar and LCD_DrawPicture functions.
* Return         : None
*******************************************************************************/
void LCD_SetBackColor(vu16 Color)
{
	BackColor = Color;
}
/*******************************************************************************
* Function Name  : LCD_ClearLine
* Description    : Clears the selected line.
* Input          : - Line: the Line to be cleared.
*                    This parameter can be one of the following values:
*                       - Linex: where x can be 0..9
* Output         : None
* Return         : None
*******************************************************************************/
/*******************************************************************************
* Function Name  : LCD_Clear
* Description    : Clears the hole LCD.
* Input          : Color: the color of the background.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_Clear(u16 Color)
{
	u32 index = 0;
	LCD_SetCursor(0x00, 0x0000); 
	LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
	for(index = 0; index < 76800; index++)
	{
		LCD_WriteRAM(Color);    
	}
}
/*******************************************************************************
* Function Name  : LCD_SetCursor
* Description    : Sets the cursor position.
* Input          : - Xpos: specifies the X position.
*                  - Ypos: specifies the Y position. 
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_SetCursor(u8 Xpos, u16 Ypos)
{ 
	LCD_WriteReg(R32, Xpos);
	LCD_WriteReg(R33, Ypos);
}
/*******************************************************************************
* Function Name  : LCD_DrawChar
* Description    : Draws a character on LCD.
* Input          : - Xpos: the Line where to display the character shape.
*                    This parameter can be one of the following values:
*                       - Linex: where x can be 0..9
*                  - Ypos: start column address.
*                  - c: pointer to the character data.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DrawChar(u8 Xpos, u16 Ypos, uc16 *c)
{
	u32 index = 0, i = 0;
	u8 Xaddress = 0;
   
	Xaddress = Xpos;
	LCD_SetCursor(Xaddress, Ypos);
  
	for(index = 0; index < 24; index++)
	{
		LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
		for(i = 0; i < 16; i++)
		{
			if((c[index] & (1 << i)) == 0x00)
			{
				LCD_WriteRAM(BackColor);
			}
			else
			{
				LCD_WriteRAM(TextColor);
			}
		}
		Xaddress++;
		LCD_SetCursor(Xaddress, Ypos);
	}
}
/*******************************************************************************
* Function Name  : LCD_DisplayChar
* Description    : Displays one character (16dots width, 24dots height).
* Input          : - Line: the Line where to display the character shape .
*                    This parameter can be one of the following values:
*                       - Linex: where x can be 0..9
*                  - Column: start column address.
*                  - Ascii: character ascii code, must be between 0x20 and 0x7E.
* Output         : None
* Return         : None
*******************************************************************************/
/*
void LCD_DisplayChar(u8 Line, u16 Column, u8 Ascii)
{
	Ascii -= 32;
	LCD_DrawChar(Line, Column, &ASCII_Table[Ascii * 24]);
}
*/
/*******************************************************************************
* Function Name  : LCD_DisplayStringLine
* Description    : Displays a maximum of 20 char on the LCD.
* Input          : - Line: the Line where to display the character shape .
*                    This parameter can be one of the following values:
*                       - Linex: where x can be 0..9
*                  - *ptr: pointer to string to display on LCD.
* Output         : None
* Return         : None
*******************************************************************************/
/*
void LCD_DisplayStringLine(u8 Line, u8 *ptr)
{
	u32 i = 0;
	u16 refcolumn = 319;//319;

	while ((*ptr != 0) && (i < 20))	 //	20
	{
		LCD_DisplayChar(Line, refcolumn, *ptr);
		refcolumn -= 16;
		ptr++;
		i++;
	}
}
*/
/*******************************************************************************
* Function Name  : LCD_SetDisplayWindow
* Description    : Sets a display window
* Input          : - Xpos: specifies the X buttom left position.
*                  - Ypos: specifies the Y buttom left position.
*                  - Height: display window height.
*                  - Width: display window width.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_SetDisplayWindow(u8 Xpos, u16 Ypos, u8 Height, u16 Width)
{
	if(Xpos >= Height)
	{
		LCD_WriteReg(R80, (Xpos - Height + 1));
	}
	else
	{
		LCD_WriteReg(R80, 0);
	}
	LCD_WriteReg(R81, Xpos);
	if(Ypos >= Width)
	{
		LCD_WriteReg(R82, (Ypos - Width + 1));
	}  
	else
	{
		LCD_WriteReg(R82, 0);
	}
	/* Vertical GRAM End Address */
	LCD_WriteReg(R83, Ypos);
	LCD_SetCursor(Xpos, Ypos);
}
/*******************************************************************************
* Function Name  : LCD_WindowModeDisable
* Description    : Disables LCD Window mode.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_WindowModeDisable(void)
{
	LCD_SetDisplayWindow(239, 0x13F, 240, 320);
	LCD_WriteReg(R3, 0x1018);    
}
/*******************************************************************************
* Function Name  : LCD_DrawLine
* Description    : Displays a line.
* Input          : - Xpos: specifies the X position.
*                  - Ypos: specifies the Y position.
*                  - Length: line length.
*                  - Direction: line direction.
*                    This parameter can be one of the following values: Vertical 
*                    or Horizontal.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DrawLine(u8 Xpos, u16 Ypos, u16 Length, u8 Direction)
{
	u32 i = 0;
  
	LCD_SetCursor(Xpos, Ypos);
	if(Direction == Horizontal)
	{
		LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
		for(i = 0; i < Length; i++)
		{
			LCD_WriteRAM(TextColor);
		}
	}
	else
	{
		for(i = 0; i < Length; i++)
		{
			LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
			LCD_WriteRAM(TextColor);
			Xpos++;
			LCD_SetCursor(Xpos, Ypos);
		}
	}
}
/*******************************************************************************
* Function Name  : LCD_DrawRect
* Description    : Displays a rectangle.
* Input          : - Xpos: specifies the X position.
*                  - Ypos: specifies the Y position.
*                  - Height: display rectangle height.
*                  - Width: display rectangle width.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DrawRect(u8 Xpos, u16 Ypos, u8 Height, u16 Width)
{
	LCD_DrawLine(Xpos, Ypos, Width, Horizontal);
	LCD_DrawLine((Xpos + Height), Ypos, Width, Horizontal);
  
	LCD_DrawLine(Xpos, Ypos, Height, Vertical);
	LCD_DrawLine(Xpos, (Ypos - Width + 1), Height, Vertical);
}
/*******************************************************************************
* Function Name  : LCD_DrawCircle
* Description    : Displays a circle.
* Input          : - Xpos: specifies the X position.
*                  - Ypos: specifies the Y position.
*                  - Height: display rectangle height.
*                  - Width: display rectangle width.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DrawCircle(u8 Xpos, u16 Ypos, u16 Radius)
{
	s32  D;
	u32  CurX;
	u32  CurY;
  
  	D = 3 - (Radius << 1);
  	CurX = 0;
  	CurY = Radius;
  
  	while (CurX <= CurY)
  	{
	    LCD_SetCursor(Xpos + CurX, Ypos + CurY);
	    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
	    LCD_WriteRAM(TextColor);
	
	    LCD_SetCursor(Xpos + CurX, Ypos - CurY);
	    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
	    LCD_WriteRAM(TextColor);
	
	    LCD_SetCursor(Xpos - CurX, Ypos + CurY);
	    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
	    LCD_WriteRAM(TextColor);
	
	    LCD_SetCursor(Xpos - CurX, Ypos - CurY);
	    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
	    LCD_WriteRAM(TextColor);
	
	    LCD_SetCursor(Xpos + CurY, Ypos + CurX);
	    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
	    LCD_WriteRAM(TextColor);
	
	    LCD_SetCursor(Xpos + CurY, Ypos - CurX);
	    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
	    LCD_WriteRAM(TextColor);
	
	    LCD_SetCursor(Xpos - CurY, Ypos + CurX);
	    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
	    LCD_WriteRAM(TextColor);
	
	    LCD_SetCursor(Xpos - CurY, Ypos - CurX);
	    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
	    LCD_WriteRAM(TextColor);
	
	    if (D < 0)
	    { 
	      D += (CurX << 2) + 6;
	    }
	    else
	    {
	      D += ((CurX - CurY) << 2) + 10;
	      CurY--;
	    }
	    CurX++;
  	}
}
/*******************************************************************************
* Function Name  : LCD_DrawMonoPict
* Description    : Displays a monocolor picture.
* Input          : - Pict: pointer to the picture array.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DrawMonoPict(uc32 *Pict)
{
	u32 index = 0, i = 0;

	LCD_SetCursor(0, 319); 

	LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
	for(index = 0; index < 2400; index++)
	{
		for(i = 0; i < 32; i++)
		{
			if((Pict[index] & (1 << i)) == 0x00)
			{
				LCD_WriteRAM(BackColor);
			}
			else
			{
				LCD_WriteRAM(TextColor);
			}
		}
	}
}
/*******************************************************************************
* Function Name  : LCD_WriteBMP
* Description    : Displays a bitmap picture loaded in the internal Flash.
* Input          : - BmpAddress: Bmp picture address in the internal Flash.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_WriteBMP(u32 BmpAddress)
{
	u32 index = 0, size = 0;

	size = *(vu16 *) (BmpAddress + 2);
	size |= (*(vu16 *) (BmpAddress + 4)) << 16;

	index = *(vu16 *) (BmpAddress + 10);
	index |= (*(vu16 *) (BmpAddress + 12)) << 16;
	size = (size - index)/2;
	BmpAddress += index;

	LCD_WriteReg(R3, 0x1008);
	LCD_WriteRAM_Prepare();
	for(index = 0; index < size; index++)
	{
		LCD_WriteRAM(*(vu16 *)BmpAddress);
		BmpAddress += 2;
	}
	LCD_WriteReg(R3, 0x1018);
}
/*******************************************************************************
* Function Name  : LCD_WriteReg
* Description    : Writes to the selected LCD register.
* Input          : - LCD_Reg: address of the selected register.
*                  - LCD_RegValue: value to write to the selected register.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
{
	GPIOB->BRR = 0x0200;  
	GPIOB->BRR = 0x0100;  
	GPIOB->BSRR = 0x0020; 

	GPIOC->ODR = LCD_Reg; 
	GPIOB->BRR = 0x0020; 
	GPIOB->BSRR = 0x0020; 
	GPIOB->BSRR = 0x0100; 

	GPIOC->ODR = LCD_RegValue; 
	GPIOB->BRR = 0x0020;   
	GPIOB->BSRR = 0x0020; 
	GPIOB->BSRR = 0x0100; 
}
/*******************************************************************************
* Function Name  : LCD_ReadReg
* Description    : Reads the selected LCD Register.
* Input          : None
* Output         : None
* Return         : LCD Register Value.
*******************************************************************************/
u16 LCD_ReadReg(u8 LCD_Reg)
{
	u16 temp;

	GPIOB->BRR = 0x0200;  
	GPIOB->BRR = 0x0100;  
	GPIOB->BSRR = 0x0020; 

	GPIOC->ODR = LCD_Reg; 
	GPIOB->BRR = 0x0020; 
	GPIOB->BSRR = 0x0020;
	GPIOB->BSRR = 0x0100;

	LCD_BusIn();
	GPIOB->BRR = 0x0400;  
	temp = GPIOC->IDR;    
	GPIOB->BSRR = 0x0400; 

	LCD_BusOut();
	GPIOB->BSRR = 0x0200; 

	return temp;
}
/*******************************************************************************
* Function Name  : LCD_WriteRAM_Prepare
* Description    : Prepare to write to the LCD RAM.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_WriteRAM_Prepare(void)
{ 
	GPIOB->BRR = 0x0200;  
	GPIOB->BRR = 0x0100; 
	GPIOB->BSRR = 0x0020; 

	GPIOC->ODR = R34;     
	GPIOB->BRR = 0x0020;   
	GPIOB->BSRR = 0x0020;
	GPIOB->BSRR = 0x0100; 

	GPIOB->BSRR = 0x0200; 
}
/*******************************************************************************
* Function Name  : LCD_WriteRAM
* Description    : Writes to the LCD RAM.
* Input          : - RGB_Code: the pixel color in RGB mode (5-6-5).
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_WriteRAM(u16 RGB_Code)
{
	GPIOB->BRR = 0x0200;  
	GPIOB->BSRR = 0x0100; 
	GPIOB->BSRR = 0x0020; 

	GPIOC->ODR = RGB_Code;
	GPIOB->BRR = 0x0020;  
	GPIOB->BSRR = 0x0020; 
	GPIOB->BSRR = 0x0100; 

	GPIOB->BSRR = 0x0200; 
}
/*******************************************************************************
* Function Name  : LCD_ReadRAM
* Description    : Reads the LCD RAM.
* Input          : None
* Output         : None
* Return         : LCD RAM Value.
*******************************************************************************/
u16 LCD_ReadRAM(void)
{
	u16 temp;

	GPIOB->BRR = 0x0200; 
	GPIOB->BRR = 0x0100; 
	GPIOB->BSRR = 0x0020; 

	GPIOC->ODR = R34;     
	GPIOB->BRR = 0x0020;  
	GPIOB->BSRR = 0x0020; 
	GPIOB->BSRR = 0x0100; 

	LCD_BusIn();
	GPIOB->BRR = 0x0400; 
	temp = GPIOC->IDR;  
	GPIOB->BSRR = 0x0400;

	LCD_BusOut();
	GPIOB->BSRR = 0x0200; 
                         
	return temp;
}
/*******************************************************************************
* Function Name  : LCD_PowerOn
* Description    : Power on the LCD.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_PowerOn(void)
{
	LCD_WriteReg(R16, 0x0000);
	LCD_WriteReg(R17, 0x0000); 
	LCD_WriteReg(R18, 0x0000);
	LCD_WriteReg(R19, 0x0000); 
	Delay_LCD(20);             
	LCD_WriteReg(R16, 0x17B0); 
	LCD_WriteReg(R17, 0x0137);
	Delay_LCD(5);             
	LCD_WriteReg(R18, 0x0139); 
	Delay_LCD(5);             
	LCD_WriteReg(R19, 0x1d00); 
	LCD_WriteReg(R41, 0x0013); 
	Delay_LCD(5);             
	LCD_WriteReg(R7, 0x0173);
}
/*******************************************************************************
* Function Name  : LCD_DisplayOn
* Description    : Enables the Display.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DisplayOn(void)
{
	LCD_WriteReg(R7, 0x0173);
}
/*******************************************************************************
* Function Name  : LCD_DisplayOff
* Description    : Disables the Display.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DisplayOff(void)
{
	LCD_WriteReg(R7, 0x0); 
}
/*******************************************************************************
* Function Name  : LCD_CtrlLinesConfig
* Description    : Configures LCD Control lines.
                   Push-Pull mode.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_CtrlLinesConfig(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	LCD_BusOut();
	GPIOB->BSRR |= 0x0620;
}
/*******************************************************************************
* Function Name  : LCD_BusIn
* Description    : Configures the Parallel interface for LCD(PortC)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_BusIn(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
/*******************************************************************************
* Function Name  : LCD_BusOut
* Description    : Configures the Parallel interface for LCD(PortC)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_BusOut(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
/*******************************************************************************
* Function Name  : LCD_DrawPicture
* Description    : Displays a 16 color picture.
* Input          : - picture: pointer to the picture array.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DrawPicture(const u8* picture)
{
	int index;
	LCD_SetCursor(0x00, 0x0000); 

	LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */

	for(index = 0; index < 76800; index++)
	{
		LCD_WriteRAM(picture[2*index+1]<<8|picture[2*index]);	
	}
}

void LCD_DrawPixel(u8 Xpos, u16 Ypos, u16 Color)
{
    LCD_SetCursor(Xpos, Ypos);    // 设置光标位置
    LCD_WriteRAM_Prepare();       // 准备写入GRAM
    LCD_WriteRAM(Color);          // 写入颜色
}

void PRO_DrawLine(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1; 
    int err = dx + dy, e2; /* error value e_xy */

    while (1) {
        LCD_DrawPixel(x0, y0, White); /* set pixel */
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
        if (e2 <= dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */
    }
}

void LCD_DrawOctagon(u8 centerX, u16 centerY, u16 radius) {
    u8 X[8];
    u16 Y[8];

    // Calculate the 8 points of the octagon with one side parallel to the horizontal axis
    for (int i = 0; i < 8; i++) {
        X[i] = centerX + radius * cos((i + 0.5) * 3.1415 / 4); // Rotate by 22.5 degrees
        Y[i] = centerY + radius * sin((i + 0.5) * 3.1415 / 4);
    }

    // Draw the 8 sides of the octagon
    for (int i = 0; i < 8; i++) {
        PRO_DrawLine(X[i], Y[i], X[(i+1) % 8], Y[(i+1) % 8]);
    }
}

void LCD_VerticalDisplay(u8 Xpos, u16 Ypos, uint16_t input[CHAR_HEIGHT]) 
{
    // Transpose the input character matrix for vertical display
    transposeMatrix(input, output);

    // Render the transposed character on the LCD
    PRO_DrawChar(Xpos, Ypos, output[0]);
}

void PRO_DrawChar(u8 Xpos, u16 Ypos, uc16 *c)
{
    u32 index = 0, i = 0;
    u8 Xaddress = Xpos;

    // Outer loop iterates over each column
    for(index = 0; index < 16; ++index) {
        LCD_SetCursor(Xaddress, Ypos);
        LCD_WriteRAM_Prepare(); 
        
        u16 upper_half = c[index * 2 + 1];  // Upper 12 bits of the column
        u16 lower_half = c[index * 2];      // Lower 12 bits of the column

        // Draw the lower 12 bits, from bottom to top
        for(i = 12; i > 0; --i) {
            if((lower_half & (1 << (i - 1))) == 0x00)
                LCD_WriteRAM(BackColor); 
            else
                LCD_WriteRAM(TextColor);  
        }

        // Move the cursor down by 12 pixels to draw the upper half
        LCD_SetCursor(Xaddress, Ypos + 12); 
        LCD_WriteRAM_Prepare();

        // Draw the upper 12 bits, from bottom to top
        for(i = 12; i > 0; --i) {
            if((upper_half & (1 << (i - 1))) == 0x00)
                LCD_WriteRAM(BackColor);
            else
                LCD_WriteRAM(TextColor);
        }

        // Move to the next column (next X position)
        Xaddress++;
    }
}

void transposeMatrix(uint16_t input[CHAR_HEIGHT], uint16_t output[CHAR_HEIGHT / 2][2]) 
{
    // Clear the output array
    for (int i = 0; i < CHAR_HEIGHT / 2; ++i) {
        output[i][0] = 0;
        output[i][1] = 0;
    }

    // Loop through each row and each column of the input matrix
    for (int i = 0; i < CHAR_HEIGHT; ++i) 
        for (int j = 0; j < CHAR_WIDTH; ++j) 
            // Check if the j-th bit of the i-th row is set
            if (input[i] & (1 << j)) 
                // Set the corresponding bit in the transposed output matrix
                if (i < 12) 
                    output[j / 2][0] |= (1 << i);  // Set bit in the first half (lower 12 bits)
                else 
                    output[j / 2][1] |= (1 << (i - 12));  // Set bit in the second half (upper 12 bits)          
}

float Filter_Distance(float new_distance)
{
    // Update filter buffer
    distance_buffer[filter_index] = new_distance;
    filter_index = (filter_index + 1) % FILTER_SIZE;

    float filtered_distance = 0;
    for (uint8_t i = 0; i < FILTER_SIZE; ++i)
        filtered_distance += distance_buffer[i];

    // Reduce the filtering quickly
    if (fabs(new_distance - filtered_distance / FILTER_SIZE) > 5.0f)
        return new_distance;

    return filtered_distance / FILTER_SIZE; // Average Filter
}

void LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    uint16_t i, j;
    LCD_SetTextColor(color);

    for (i = 0; i < height; ++i)
        for (j = 0; j < width; ++j)
            LCD_DrawPixel(x + j, y + i, color);
}

void LCD_Back_Init()
{
    STM3210B_LCD_Init();
    LCD_SetTextColor(White);
    LCD_SetBackColor(Black);
    LCD_Clear(Black);

    LCD_VerticalDisplay(20, 40, FontMartix['D' - 32]);
    LCD_VerticalDisplay(30, 40, FontMartix['I' - 32]);
    LCD_VerticalDisplay(40, 40, FontMartix['S' - 32]);
    LCD_VerticalDisplay(30, 60, FontMartix['T' - 32]);
    LCD_VerticalDisplay(40, 60, FontMartix['A' - 32]);
    LCD_VerticalDisplay(50, 60, FontMartix['N' - 32]);
    LCD_VerticalDisplay(60, 60, FontMartix['C' - 32]);
    LCD_VerticalDisplay(70, 60, FontMartix['E' - 32]);

    LCD_VerticalDisplay(160, 40, FontMartix['D' - 32]);
    LCD_VerticalDisplay(170, 40, FontMartix['U' - 32]);
    LCD_VerticalDisplay(180, 40, FontMartix['T' - 32]);
    LCD_VerticalDisplay(190, 40, FontMartix['Y' - 32]);
    LCD_VerticalDisplay(170, 60, FontMartix['C' - 32]);
    LCD_VerticalDisplay(180, 60, FontMartix['Y' - 32]);
    LCD_VerticalDisplay(190, 60, FontMartix['C' - 32]);
    LCD_VerticalDisplay(200, 60, FontMartix['L' - 32]);
    LCD_VerticalDisplay(210, 60, FontMartix['E' - 32]);
}
