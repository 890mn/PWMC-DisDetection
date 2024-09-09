#ifndef __LCD_H
#define __LCD_H

#include "stm32f10x.h"
#include "stdint.h"

#define CHAR_WIDTH  16 
#define CHAR_HEIGHT 24 

/* LCD Registers */
#define R0             0x00
#define R1             0x01
#define R2             0x02
#define R3             0x03
#define R4             0x04
#define R5             0x05
#define R6             0x06
#define R7             0x07
#define R8             0x08
#define R9             0x09
#define R10            0x0A
#define R12            0x0C
#define R13            0x0D
#define R14            0x0E
#define R15            0x0F
#define R16            0x10
#define R17            0x11
#define R18            0x12
#define R19            0x13
#define R20            0x14
#define R21            0x15
#define R22            0x16
#define R23            0x17
#define R24            0x18
#define R25            0x19
#define R26            0x1A
#define R27            0x1B
#define R28            0x1C
#define R29            0x1D
#define R30            0x1E
#define R31            0x1F
#define R32            0x20
#define R33            0x21
#define R34            0x22
#define R36            0x24
#define R37            0x25
#define R40            0x28
#define R41            0x29
#define R43            0x2B
#define R45            0x2D
#define R48            0x30
#define R49            0x31
#define R50            0x32
#define R51            0x33
#define R52            0x34
#define R53            0x35
#define R54            0x36
#define R55            0x37
#define R56            0x38
#define R57            0x39
#define R59            0x3B
#define R60            0x3C
#define R61            0x3D
#define R62            0x3E
#define R63            0x3F
#define R64            0x40
#define R65            0x41
#define R66            0x42
#define R67            0x43
#define R68            0x44
#define R69            0x45
#define R70            0x46
#define R71            0x47
#define R72            0x48
#define R73            0x49
#define R74            0x4A
#define R75            0x4B
#define R76            0x4C
#define R77            0x4D
#define R78            0x4E
#define R79            0x4F
#define R80            0x50
#define R81            0x51
#define R82            0x52
#define R83            0x53
#define R96            0x60
#define R97            0x61
#define R106           0x6A
#define R118           0x76
#define R128           0x80
#define R129           0x81
#define R130           0x82
#define R131           0x83
#define R132           0x84
#define R133           0x85
#define R134           0x86
#define R135           0x87
#define R136           0x88
#define R137           0x89
#define R139           0x8B
#define R140           0x8C
#define R141           0x8D
#define R143           0x8F
#define R144           0x90
#define R145           0x91
#define R146           0x92
#define R147           0x93
#define R148           0x94
#define R149           0x95
#define R150           0x96
#define R151           0x97
#define R152           0x98
#define R153           0x99
#define R154           0x9A
#define R157           0x9D
#define R192           0xC0
#define R193           0xC1
#define R227           0xE3
#define R229           0xE5
#define R231           0xE7
#define R239           0xEF

/* LCD Control pins */
//#define CtrlPin_NCS    GPIO_Pin_9   /* PB.9 */
//#define CtrlPin_RS     GPIO_Pin_8   /* PB.8 */
//#define CtrlPin_NWR    GPIO_Pin_5  /* Pb.5 */
//#define CtrlPin_NRD    GPIO_Pin_10  /* Pb.10 */

/* LCD color */
#define White          0xFFFF
#define Black          0x0000
#define Grey           0xF7DE
#define Blue           0x001F
#define Blue2          0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x7FFF
#define Yellow         0xFFE0

#define Line0          0
#define Line1          24
#define Line2          48
#define Line3          72
#define Line4          96
#define Line5          120
#define Line6          144
#define Line7          168
#define Line8          192
#define Line9          216

#define Horizontal     0x00
#define Vertical       0x01

/* Exported functions ------------------------------------------------------- */
void STM3210B_LCD_Init(void);
void LCD_SetTextColor(vu16 Color);
void LCD_SetBackColor(vu16 Color);
void LCD_ClearLine(u8 Line);
void LCD_Clear(u16 Color);
void LCD_SetCursor(u8 Xpos, u16 Ypos);

/**
 * @brief Draw a pixel at the specified coordinates with the given color.
 * 
 * @param Xpos The X-coordinate of the pixel.
 * @param Ypos The Y-coordinate of the pixel.
 * @param Color The color of the pixel (16-bit RGB value).
 */
void LCD_DrawPixel(u8 Xpos, u16 Ypos, u16 Color);

/**
 * @brief Draw a line between two points using the Bresenham's line algorithm.
 * 
 * @param x0 The X-coordinate of the start point.
 * @param y0 The Y-coordinate of the start point.
 * @param x1 The X-coordinate of the end point.
 * @param y1 The Y-coordinate of the end point.
 */
void PRO_DrawLine(int x0, int y0, int x1, int y1);

/**
 * @brief Draw an octagon centered at the specified coordinates with a given radius.
 * 
 * This function calculates the 8 vertices of an octagon with one side parallel to 
 * the horizontal axis. 
 * 
 * @param centerX The X-coordinate of the center of the octagon.
 * @param centerY The Y-coordinate of the center of the octagon.
 * @param radius The radius of the octagon.
 */
void LCD_DrawOctagon(u8 centerX, u16 centerY, u16 radius);

/**
 * @brief Display a character in vertical mode on the LCD.
 * 
 * @param Xpos The starting X coordinate for the character.
 * @param Ypos The starting Y coordinate for the character.
 * @param input The input character matrix (24x16).
 */
void LCD_VerticalDisplay(u8 Xpos, u16 Ypos, uint16_t input[CHAR_HEIGHT]);

/**
 * @brief Transpose a character matrix for vertical display.
 * 
 * This function takes a horizontally-aligned character matrix (input) and transposes it 
 * for vertical display. 
 * 
 * Each column is split into two 12-bit halves for drawing.
 * 
 * @param input The input character matrix (24x16).
 * @param output The transposed output matrix (each element contains 12 bits for high and low parts).
 */
void transposeMatrix(uint16_t input[CHAR_HEIGHT], uint16_t output[CHAR_HEIGHT / 2][2]);

/**
 * @brief Draw a character on the LCD screen.
 * 
 * output[i][0] -> low pixel in i's column  
 * 
 * output[i][1] -> high pixel in i's column 
 * 
 * Draw from low 12-bit to high 12-bit
 * 
 * @param Xpos The starting X coordinate for the character.
 * @param Ypos The starting Y coordinate for the character.
 * @param c A pointer to the character data (16 columns, 12 + 12 bits per column).
 */
void PRO_DrawChar(u8 Xpos, u16 Ypos, uc16 *c);

/**
 * @brief Init the Distance and DutyCycle's Display with presets
 * 
 */
void LCD_Back_Init(void);

/**
 * @brief Filter used for LCD Display in quick refresh mode.
 * 
 * This function filters the input distance value for display on an LCD in a mode
 * that prioritizes quick refresh rates.
 *
 * @param new_distance The new distance value to be filtered (float).
 * @return The filtered distance value (float).
 */
float Filter_Distance(float new_distance);

/**
 * @brief Filling a matrix bit-by-bit
 * 
 * @param x The X-coordinate of the start point.
 * @param y The Y-coordinate of the start point.
 * @param width The width of the martix.
 * @param height The height of the martix.
 * @param color The color of the martix.
 */
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);

void LCD_DrawChar(u8 Xpos, u16 Ypos, uc16 *c);
void LCD_SetDisplayWindow(u8 Xpos, u16 Ypos, u8 Height, u16 Width);
void LCD_WindowModeDisable(void);
void LCD_DrawRect(u8 Xpos, u16 Ypos, u8 Height, u16 Width);
void LCD_DrawCircle(u8 Xpos, u16 Ypos, u16 Radius);
void LCD_DrawMonoPict(uc32 *Pict);
void LCD_WriteBMP(u32 BmpAddress);
void LCD_DrawBMP(u32 BmpAddress);
void LCD_DrawPicture(const u8* picture);

void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue);
u16 LCD_ReadReg(u8 LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(u16 RGB_Code);
u16 LCD_ReadRAM(void);
void LCD_PowerOn(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);

void LCD_CtrlLinesConfig(void);
void LCD_BusIn(void);
void LCD_BusOut(void);

#endif
