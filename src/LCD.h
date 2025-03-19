

#ifndef __LCD
#define __LCD

#include "stm32f10x.h"

typedef enum
{ CMD_RS = 0,
  DATA_RS
}RS_STATE;

#define LCD_PWR_PIN   GPIO_Pin_6 //D
#define LCD_E_PIN     GPIO_Pin_2  //G  
#define LCD_RS_PIN    GPIO_Pin_0  //D
#define LCD_RW_PIN    GPIO_Pin_1  //D
#define LCD_D0_PIN   GPIO_Pin_4 //G
#define LCD_D1_PIN    GPIO_Pin_5 //G
#define LCD_D2_PIN    GPIO_Pin_6 //G
#define LCD_D3_PIN    GPIO_Pin_7 //G

#define LCD_RW_PORT   GPIOB
#define LCD_RS_PORT   GPIOB
#define LCD_PWR_PORT   GPIOA
#define LCD_E_PORT     GPIOB
#define LCD_DATA_PORT  GPIOC


#define SECONDLINE_START_INDEX 0xAA


#define LCD_LINE_DISPLAY_COUNT 12 //number of visible chars on one linelcd

#define MAX_CONTRAST_VALUE 0x3f
#define DEFAULT_CONTRAST_VALUE (MAX_CONTRAST_VALUE/2)

void LCD_On(void);
void LCD_sendbyte(uint8_t LCD_data, RS_STATE FLAG_RS);
void LCD_sendnibble(uint8_t LCD_data, RS_STATE FLAG_RS);
void write_LCD_lines(uint8_t* string1,uint8_t* string2);
void write_LCD_line1(uint8_t *string);
void write_LCD_line2(uint8_t *string);
void write_LCD_char(uint8_t chr);
void write_LCD(uint8_t *string);
void clear_LCD();
void LCD_Off(void);
void LCD_out_hex(char byte);
void LCD_Addr(uint8_t ADDR);


//rj
void LCD_WriteField( uint8_t bAddr, uint8_t *s, uint8_t chptr, uint8_t  NumOfField );

void LCD_ClearField ( uint8_t bAddr, uint8_t  NumOfField);
void LCD_CursorOn();
void LCD_CursorOff();

int LCD_ContrastAdjust(int offset);


#endif