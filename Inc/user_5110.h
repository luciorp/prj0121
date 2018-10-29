#ifndef __USER_5110_H_
#define __USER_5110_H_

#include "main.h"
#include "stm32f1xx_hal.h" // trocar o include conforme a familia stm32f0xx_hal.h para o M0 e stm32f3xx_hal.h para o M4

extern SPI_HandleTypeDef hspi2; // Usar o mesmo nome da struct utilizada na config do SPI utilizado

#define LCD_RESET_PIN LCD_RST_Pin // Usar o nome do pino de reset configurado.
#define LCD_RESET_PORT LCD_RST_GPIO_Port  // Usar o nome da porta de reset configurado.

#define LCD_CE_PIN LCD_CS_Pin // Usar o nome do pino de chip enable configurado
#define LCD_CE_PORT LCD_CS_GPIO_Port // Usar o nome da porta de chip enable configurado

#define LCD_DC_PIN LCD_DC_Pin // Usar o nome do pino de data/comando configurado
#define LCD_DC_PORT LCD_DC_GPIO_Port // Usar o nome da porta de data/comando configurado

#define LCD_SPI_INTERFACE hspi2 // usar a struct da variavel extern


void LCD_Init(void);
void LCD_Reset(void);
void LCD_Write_Command(uint8_t cmd);
void LCD_Write_Data(uint8_t data);
void LCD_Set_Postion(uint8_t PosX, uint8_t PosY);
void LCD_Clear(void);
void LCD_Write_Char(uint8_t ch);
void LCD_Write_String(uint8_t PosX, uint8_t PosY, char * str);

#endif
