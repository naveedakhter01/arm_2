// LCD driver routines for STM32 board


#include "LCD.h"
#include "stm32f10x.h"
#include "main.h"

//proto's
void LCD_GPIO_Config(FunctionalState NewState);


#define LCD_RESET_ADDR	((uint8_t)0x80)
#define LCD_RESET_DATA	((uint8_t)0x01)
#define LCD_LOWER_ADDR	((uint8_t)0xA8)



#define BIT0_MASK     ((uint8_t)0x01)
#define BIT1_MASK     ((uint8_t)0x02)
#define BIT2_MASK     ((uint8_t)0x04)
#define BIT3_MASK     ((uint8_t)0x08)
#define BIT4_MASK     ((uint8_t)0x10)
#define BIT5_MASK     ((uint8_t)0x20)
#define BIT6_MASK     ((uint8_t)0x40)
#define BIT7_MASK     ((uint8_t)0x80)



#define DELAY_POWER 100
#define DELAY_COMMAND 25
#define DELAY_BIT 4




void LCD_GPIO_Config(FunctionalState NewState)
{
  GPIO_InitTypeDef GPIO_InitStructure; 
   
  if (NewState == ENABLE)
  {
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);  
    
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        //set ouitputs data lines
        GPIO_InitStructure.GPIO_Pin = LCD_D0_PIN | LCD_D1_PIN | LCD_D2_PIN | LCD_D3_PIN;
        GPIO_Init(LCD_DATA_PORT, &GPIO_InitStructure);
        //set ouitputs cntrl lines
        GPIO_InitStructure.GPIO_Pin = LCD_RW_PIN;
        GPIO_Init(LCD_RW_PORT, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = LCD_RS_PIN;
        GPIO_Init(LCD_RS_PORT, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = LCD_E_PIN;
        GPIO_Init(LCD_E_PORT, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = LCD_PWR_PIN;
        GPIO_Init(LCD_PWR_PORT, &GPIO_InitStructure);
  }
  else //DISABLE LCD pins - turn all outputs to analog inputs to save power
  {
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        //set ouitputs data lines
        GPIO_InitStructure.GPIO_Pin = LCD_D0_PIN | LCD_D1_PIN | LCD_D2_PIN | LCD_D3_PIN;
        GPIO_Init(LCD_DATA_PORT, &GPIO_InitStructure);
        //set ouitputs cntrl lines
        GPIO_InitStructure.GPIO_Pin = LCD_RW_PIN;
        GPIO_Init(LCD_RW_PORT, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = LCD_RS_PIN;
        GPIO_Init(LCD_RS_PORT, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = LCD_E_PIN;
        GPIO_Init(LCD_E_PORT, &GPIO_InitStructure);
        
        //except of course the enable pin for the 5v boost reg 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Pin = LCD_PWR_PIN;
        GPIO_Init(LCD_PWR_PORT, &GPIO_InitStructure);
        
       // GPIO_WriteBit(LCD_PWR_PORT,LCD_PWR_PIN, Bit_RESET);
  }
}


void LCD_On(void)
{
  LCD_GPIO_Config(ENABLE); //enable gpio ports
  
  GPIO_WriteBit(LCD_PWR_PORT,LCD_PWR_PIN, Bit_SET);   // let the LCD power up
  
  delay_us(DELAY_POWER*10);			
  
  for (int i=0;i<3;i++) 
  {
    LCD_sendnibble((uint8_t)0x03,CMD_RS);
    delay_us(200);
  }
  // LCD is reset
  LCD_sendnibble((uint8_t)0x02,CMD_RS);		// set to 4 bit mode
  delay_us(DELAY_COMMAND);
  //LCD_WaitBF();
  LCD_sendbyte((uint8_t)0x2c,CMD_RS);
  delay_us(DELAY_COMMAND);
  LCD_sendbyte((uint8_t)0x08,CMD_RS);	
  delay_us(DELAY_COMMAND);
  LCD_sendbyte((uint8_t)0x01,CMD_RS);		
  delay_us(150);
  LCD_sendbyte((uint8_t)0x06,CMD_RS);	
  delay_us(DELAY_COMMAND);
  LCD_sendbyte((uint8_t)0x0c,CMD_RS);
  delay_us(DELAY_COMMAND);
  LCD_sendbyte(LCD_RESET_ADDR,CMD_RS);		
  delay_us(DELAY_COMMAND*3);
}





void LCD_off(void)
{
  LCD_GPIO_Config(DISABLE); //enable gpio ports
  
  GPIO_WriteBit(LCD_PWR_PORT,LCD_PWR_PIN, Bit_RESET);   // turn off bosst reg
}




void LCD_sendbyte(uint8_t LCD_data, RS_STATE FLAG_RS)
{
  
  
        GPIO_WriteBit(LCD_RS_PORT,LCD_RS_PIN, (BitAction)(FLAG_RS));  
        GPIO_WriteBit(LCD_RW_PORT,LCD_RW_PIN, Bit_RESET);   
	// put the high nibble onto the PORT
          
        GPIO_WriteBit(LCD_DATA_PORT,LCD_D0_PIN, (BitAction)(LCD_data & BIT4_MASK));    
        GPIO_WriteBit(LCD_DATA_PORT,LCD_D1_PIN, (BitAction)(LCD_data & BIT5_MASK));
        GPIO_WriteBit(LCD_DATA_PORT,LCD_D2_PIN, (BitAction)(LCD_data & BIT6_MASK));
        GPIO_WriteBit(LCD_DATA_PORT,LCD_D3_PIN, (BitAction)(LCD_data & BIT7_MASK)); 
          

	// clock it in
	delay_us(DELAY_BIT);
        GPIO_WriteBit(LCD_E_PORT,LCD_E_PIN, Bit_SET);
	
	delay_us(DELAY_BIT);
        GPIO_WriteBit(LCD_E_PORT,LCD_E_PIN, Bit_RESET);	
	// put the low nibble on
        GPIO_WriteBit(LCD_RS_PORT,LCD_RS_PIN, (BitAction)(FLAG_RS));
	GPIO_WriteBit(LCD_RW_PORT,LCD_RW_PIN, Bit_RESET);

	// put the low nibble onto the PORT	
        GPIO_WriteBit(LCD_DATA_PORT,LCD_D0_PIN, (BitAction)(LCD_data & BIT0_MASK));    
        GPIO_WriteBit(LCD_DATA_PORT,LCD_D1_PIN, (BitAction)(LCD_data & BIT1_MASK));
        GPIO_WriteBit(LCD_DATA_PORT,LCD_D2_PIN, (BitAction)(LCD_data & BIT2_MASK));
        GPIO_WriteBit(LCD_DATA_PORT,LCD_D3_PIN, (BitAction)(LCD_data & BIT3_MASK));
        
	// clock it in
	delay_us(DELAY_BIT);
        GPIO_WriteBit(LCD_E_PORT,LCD_E_PIN, Bit_SET);
	delay_us(DELAY_BIT);
        GPIO_WriteBit(LCD_E_PORT,LCD_E_PIN, Bit_RESET);	
}

void LCD_sendnibble(uint8_t LCD_data, RS_STATE FLAG_RS)
{
        GPIO_WriteBit(LCD_RS_PORT,LCD_RS_PIN, (BitAction)(FLAG_RS));  
        GPIO_WriteBit(LCD_RW_PORT,LCD_RW_PIN, Bit_RESET);  
	// put the high nibble onto the PORT	
        GPIO_WriteBit(LCD_DATA_PORT,LCD_D0_PIN, (BitAction)(LCD_data & BIT0_MASK));    
        GPIO_WriteBit(LCD_DATA_PORT,LCD_D1_PIN, (BitAction)(LCD_data & BIT1_MASK));
        GPIO_WriteBit(LCD_DATA_PORT,LCD_D2_PIN, (BitAction)(LCD_data & BIT2_MASK));
        GPIO_WriteBit(LCD_DATA_PORT,LCD_D3_PIN, (BitAction)(LCD_data & BIT3_MASK));
	// clock it in
	delay_us(DELAY_BIT);
        GPIO_WriteBit(LCD_E_PORT,LCD_E_PIN, Bit_SET);
	delay_us(DELAY_BIT);
        GPIO_WriteBit(LCD_E_PORT,LCD_E_PIN, Bit_RESET);	
}


void write_LCD_lines(uint8_t *string1,uint8_t *string2)
{
	uint8_t t;
		
	LCD_sendbyte(LCD_RESET_DATA,CMD_RS);
	delay_us(100);
	LCD_sendbyte(LCD_RESET_ADDR,CMD_RS);
	delay_us(100);
	
	for (t=0;t<16;t++) 
		{
			if (*string1 == (char)0x00) break;
			LCD_sendbyte(*string1,DATA_RS);
                        string1++;
		}
		// set address to start of lower line
		LCD_sendbyte(LCD_LOWER_ADDR,CMD_RS);
	for (t=0;t<16;t++)
		{
			if (*string2 == (char)0x00) break;
			LCD_sendbyte(*string2,DATA_RS);
                        string2++;
		}
	
}

void write_LCD_line1(uint8_t *string)
{
	uint8_t t;
        
	LCD_sendbyte(LCD_RESET_ADDR,CMD_RS);
	delay_us(100);
	
	for (t=0;t<16;t++) 
		{
			if (*string == (char)0x00) break;
			LCD_sendbyte(*string,DATA_RS);
                        string++;
		}	
}
void write_LCD_line2(uint8_t *string)
{
	uint8_t t;
	
	// set address to start of lower line
	LCD_sendbyte(LCD_LOWER_ADDR,CMD_RS);
        delay_us(100);
	for (t=0;t<16;t++)
		{
			if (*string == (char)0x00) break;
			LCD_sendbyte(*string,DATA_RS);
                        string++;
		}	
}

void write_LCD(uint8_t *string)
{
  uint8_t t;
	for (t=0;t<16;t++)
		{
			if (*string == (char)0x00) break;
			LCD_sendbyte(*string,DATA_RS);
                        string++;
		}	   
}

void write_LCD_char(uint8_t chr)
{
  
 LCD_sendbyte(chr,DATA_RS); 
  
}


void clear_LCD()
{
	LCD_sendbyte(LCD_RESET_DATA,CMD_RS);
	delay_us(100);
	LCD_sendbyte(LCD_RESET_ADDR,CMD_RS);
	delay_us(100);
}



void LCD_out_hex(char byte)
{
  
   char digit;


   
   digit = (((byte & 0xf0) >> 4) <= 9) ? (((byte & 0xf0) >> 4) + 0x30) : (((byte & 0xf0) >> 4) + 0x37);
   LCD_sendbyte(digit,DATA_RS); 
   digit = ((byte & 0x0f) <= 9) ? ((byte & 0x0f) + 0x30) : ((byte & 0x0f) + 0x37);
   LCD_sendbyte(digit,DATA_RS);      
  
}


void LCD_Addr(uint8_t ADDR)
{
  
  	LCD_sendbyte(ADDR,CMD_RS);
        delay_us(DELAY_COMMAND);
  
  
  
}


void LCD_CursorOn()
{
	LCD_sendbyte(LCD_LOWER_ADDR,CMD_RS);
        delay_us(200);

	LCD_sendbyte (0x0E, CMD_RS);
	delay_us(200);

}


void LCD_CursorOff()
{
	LCD_sendbyte(LCD_LOWER_ADDR,CMD_RS);
        delay_us(200);

	LCD_sendbyte (0x0C, CMD_RS);
	delay_us(200);

}



void LCD_WriteField( uint8_t bAddr, uint8_t *s, uint8_t chptr, uint8_t  NumOfField )
{
    int i;
   	 
        
	LCD_sendbyte(bAddr ,CMD_RS);	
	for (i=0;i<NumOfField;i++)
	{
		if (s[chptr] == (char)0x00) break;
		LCD_sendbyte(s[chptr],DATA_RS);
							chptr++;
	}	
}

void LCD_ClearField ( uint8_t bAddr, uint8_t  NumOfField)
{
	int i;

	LCD_sendbyte(bAddr ,CMD_RS);	
	for ( i=0; i< NumOfField; i++)
	{
		LCD_sendbyte(0x20,DATA_RS);
	}
  
} 






















