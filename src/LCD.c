// LCD driver routines for STM32 board


#include "LCD.h"
#include "stm32f10x.h"
#include "main.h"
#include "eeprom.h"




//proto's
void LCD_GPIO_Config(FunctionalState NewState);

//#define INVERT_E

#define LCD_RESET_ADDR	((uint8_t)0x80)
#define LCD_RESET_DATA	((uint8_t)0x01)
#define LCD_LOWER_ADDR	((uint8_t)0xC0)



#define BIT0_MASK     ((uint8_t)0x01)
#define BIT1_MASK     ((uint8_t)0x02)
#define BIT2_MASK     ((uint8_t)0x04)
#define BIT3_MASK     ((uint8_t)0x08)
#define BIT4_MASK     ((uint8_t)0x10)
#define BIT5_MASK     ((uint8_t)0x20)
#define BIT6_MASK     ((uint8_t)0x40)
#define BIT7_MASK     ((uint8_t)0x80)



#define DELAY_POWER 400
#define DELAY_COMMAND 20 //40 80
#define DELAY_BIT 8   //8   32

#define DELAY_LINE_MS 2  //1 3





int32_t contrast_value = DEFAULT_CONTRAST_VALUE; //6bits used for contrast  0x00-0x3F


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
        
        
#ifdef INVERT_E
        GPIO_WriteBit(LCD_E_PORT,LCD_E_PIN, Bit_SET);
#else	
        GPIO_WriteBit(LCD_E_PORT,LCD_E_PIN, Bit_RESET);	        
#endif

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


void LCD_SwitchPinDir(PinDirection direction)
{
  GPIO_InitTypeDef GPIO_InitStructure; 
   
  if (direction == INPUT)
  {
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        //set ouitputs data lines
        GPIO_InitStructure.GPIO_Pin = LCD_D0_PIN | LCD_D1_PIN | LCD_D2_PIN | LCD_D3_PIN;
        GPIO_Init(LCD_DATA_PORT, &GPIO_InitStructure); 
  }
  else
  {
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        //set ouitputs data lines
        GPIO_InitStructure.GPIO_Pin = LCD_D0_PIN | LCD_D1_PIN | LCD_D2_PIN | LCD_D3_PIN;
        GPIO_Init(LCD_DATA_PORT, &GPIO_InitStructure);
  }
}
       




uint8_t LCD_WaitBusy()
{
  bool busyFlag = TRUE;
  LCD_SwitchPinDir(INPUT);
  delay_us(200);
  uint8_t address = 0;
  
  GPIO_WriteBit(LCD_RS_PORT,LCD_RS_PIN, Bit_RESET);  
  GPIO_WriteBit(LCD_RW_PORT,LCD_RW_PIN, Bit_SET); 
  
  do 
  {
    int i = 2;  //loop 2 times because 4bit instructions
    address = 0;
    while (i > 0)
    {
          GPIO_WriteBit(LCD_E_PORT,LCD_E_PIN, Bit_SET);     
          delay_us(DELAY_BIT);
          if (i==2)
          {
            busyFlag =  (GPIO_ReadInputDataBit(LCD_DATA_PORT, LCD_D3_PIN) != 0) ? TRUE: FALSE; 
            address |=   GPIO_ReadInputDataBit(LCD_DATA_PORT, LCD_D2_PIN) != 0 ?  0x40 : 0;
            address |=   GPIO_ReadInputDataBit(LCD_DATA_PORT, LCD_D1_PIN) != 0 ?  0x20 : 0;
            address |=   GPIO_ReadInputDataBit(LCD_DATA_PORT, LCD_D0_PIN) != 0 ?  0x10 : 0;
          }
          else
          {
            address |=   GPIO_ReadInputDataBit(LCD_DATA_PORT, LCD_D3_PIN) != 0 ?  0x08 : 0;
            address |=   GPIO_ReadInputDataBit(LCD_DATA_PORT, LCD_D2_PIN) != 0 ?  0x04 : 0;
            address |=   GPIO_ReadInputDataBit(LCD_DATA_PORT, LCD_D1_PIN) != 0 ?  0x02 : 0;
            address |=   GPIO_ReadInputDataBit(LCD_DATA_PORT, LCD_D0_PIN) != 0 ?  0x01 : 0;
          }
           
          
          
          GPIO_WriteBit(LCD_E_PORT,LCD_E_PIN, Bit_RESET);
          delay_us(DELAY_BIT);
           
          i--;
    }
  //check if pin is high (= busy)
  }
  while (busyFlag);
   GPIO_WriteBit(LCD_RW_PORT,LCD_RW_PIN, Bit_RESET);          
   
   
  LCD_SwitchPinDir(OUTPUT);
  delay_us(200);

 
  
  return address;
}





/*
void LCD_On(void)
{
  LCD_GPIO_Config(ENABLE); //enable gpio ports
  
  GPIO_WriteBit(LCD_PWR_PORT,LCD_PWR_PIN, Bit_SET);   // let the LCD power up
  
  delay_ms(DELAY_POWER);			
  
  for (int i=0;i<3;i++) 
  {
    LCD_sendnibble((uint8_t)0x03,CMD_RS);
    delay_ms(6);
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
  delay_ms(3);
  LCD_sendbyte((uint8_t)0x06,CMD_RS);	
  delay_us(DELAY_COMMAND);
  LCD_sendbyte((uint8_t)0x0c,CMD_RS);
  delay_us(DELAY_COMMAND);
  LCD_sendbyte(LCD_RESET_ADDR,CMD_RS);		
  delay_ms(DELAY_COMMAND);
}
*/

void LCD_On(void)
{
  LCD_GPIO_Config(ENABLE); //enable gpio ports
  
  GPIO_WriteBit(LCD_PWR_PORT,LCD_PWR_PIN, Bit_SET);   // let the LCD power up
   
  delay_ms(DELAY_POWER);	
  contrast_value = EEPROM_ReadContrast();
  
  for (int i=0;i<3;i++) 
  {
    LCD_sendnibble((uint8_t)0x03,CMD_RS);
    delay_ms(6);
  }
  // LCD is reset
  LCD_sendnibble((uint8_t)0x02,CMD_RS);		// set to 4 bit mode
  delay_us(DELAY_COMMAND);
  //LCD_WaitBF();
  LCD_sendbyte((uint8_t)0x29,CMD_RS); // 4bit instrct tble 1
  delay_us(DELAY_COMMAND);
  
  
    //bias
  LCD_sendbyte((uint8_t)0x1C,CMD_RS);
  delay_us(DELAY_COMMAND);
  
  //contrast MSB
  //  LCD_sendbyte((uint8_t)0x55,CMD_RS);   //for 3.3v vcc.  booster on
 // delay_us(DELAY_COMMAND);  
  
  
  //voltage follower
  LCD_sendbyte((uint8_t)0x6C,CMD_RS); //69 $6A for 5v $6D for 3.3v
  delay_us(DELAY_COMMAND);
  
  //contrast LSB
  LCD_sendbyte((uint8_t)(0x50 | ((contrast_value>>4) & 0x0003)) ,CMD_RS);  //0x54 for 3.3v supply
  delay_us(DELAY_COMMAND);
  //lsb 4 bits
  LCD_sendbyte((uint8_t)(0x70 | (contrast_value & 0x000f)), CMD_RS);
  delay_us(DELAY_COMMAND);
  
  
  //function set 4bit instrct tble 0
  LCD_sendbyte((uint8_t)0x28,CMD_RS);
  delay_us(DELAY_COMMAND);
  
  LCD_sendbyte((uint8_t)0x08,CMD_RS);	
  delay_us(DELAY_COMMAND);
  LCD_sendbyte((uint8_t)0x01,CMD_RS);		
  delay_ms(3);
  LCD_sendbyte((uint8_t)0x06,CMD_RS);	
  delay_us(DELAY_COMMAND);
  LCD_sendbyte((uint8_t)0x0c,CMD_RS);
  delay_us(DELAY_COMMAND);
  LCD_sendbyte(LCD_RESET_ADDR,CMD_RS);		
  delay_ms(DELAY_COMMAND);
}


void LCD_Off(void)
{
  LCD_GPIO_Config(DISABLE); //enable gpio ports
  
  GPIO_WriteBit(LCD_PWR_PORT,LCD_PWR_PIN, Bit_RESET);   // turn off bosst reg
}


//contrast adjustment for lcd dogm162w
int LCD_ContrastAdjust(int offset)
{
  //get current contrast value
  contrast_value += offset;
  contrast_value = contrast_value > MAX_CONTRAST_VALUE ? MAX_CONTRAST_VALUE : contrast_value; //max 
  contrast_value = contrast_value < 0x0 ? 0x0 : contrast_value; //min 
  
  if (offset != 0)
  {
    LCD_sendbyte((uint8_t)0x29,CMD_RS); // 4bit instrct tble 1
    delay_us(DELAY_COMMAND);
      //msbits 2bits
    //LCD_sendbyte((uint8_t)(0x50 | ((contrast_value>>4) & 0x0003)) ,CMD_RS);   //5v supply
    //delay_us(DELAY_COMMAND);
    //msbits 2bits
    LCD_sendbyte((uint8_t)(0x54 | ((contrast_value>>4) & 0x0003)) ,CMD_RS);  //3.3v supply
    delay_us(DELAY_COMMAND);
    //lsb 4 bits
    LCD_sendbyte((uint8_t)(0x70 | (contrast_value & 0x000f)), CMD_RS);
    delay_us(DELAY_COMMAND);
    
    
    //function set 4bit instrct tble 0
    LCD_sendbyte((uint8_t)0x28,CMD_RS);
    delay_us(DELAY_COMMAND);
    
    EEPROM_WriteContrast(contrast_value);
  }
  return contrast_value;
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
	delay_us(DELAY_BIT);          
        
	// clock it in
        GPIO_WriteBit(LCD_E_PORT,LCD_E_PIN, Bit_SET);
	delay_us(DELAY_BIT);
        GPIO_WriteBit(LCD_E_PORT,LCD_E_PIN, Bit_RESET);	
        delay_us(DELAY_BIT);

        
        
	// put the low nibble on
        GPIO_WriteBit(LCD_RS_PORT,LCD_RS_PIN, (BitAction)(FLAG_RS));
	GPIO_WriteBit(LCD_RW_PORT,LCD_RW_PIN, Bit_RESET);

	// put the low nibble onto the PORT	
        GPIO_WriteBit(LCD_DATA_PORT,LCD_D0_PIN, (BitAction)(LCD_data & BIT0_MASK));    
        GPIO_WriteBit(LCD_DATA_PORT,LCD_D1_PIN, (BitAction)(LCD_data & BIT1_MASK));
        GPIO_WriteBit(LCD_DATA_PORT,LCD_D2_PIN, (BitAction)(LCD_data & BIT2_MASK));
        GPIO_WriteBit(LCD_DATA_PORT,LCD_D3_PIN, (BitAction)(LCD_data & BIT3_MASK));
	delay_us(DELAY_BIT);        

	// clock it in
        GPIO_WriteBit(LCD_E_PORT,LCD_E_PIN, Bit_SET);
	delay_us(DELAY_BIT);
        
        
        GPIO_WriteBit(LCD_E_PORT,LCD_E_PIN, Bit_RESET);	
        delay_us(DELAY_BIT);    
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
	delay_us(DELAY_BIT);        
        
	// clock it in
        GPIO_WriteBit(LCD_E_PORT,LCD_E_PIN, Bit_SET);
	delay_us(DELAY_BIT);
        
        GPIO_WriteBit(LCD_E_PORT,LCD_E_PIN, Bit_RESET);	
        delay_us(DELAY_BIT);    
}


void write_LCD_lines(uint8_t *string1,uint8_t *string2)
{
	uint8_t t;
		
	LCD_sendbyte(LCD_RESET_DATA,CMD_RS);
	delay_ms(DELAY_LINE_MS);
	LCD_sendbyte(LCD_RESET_ADDR,CMD_RS);
	delay_ms(DELAY_LINE_MS);
	
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
        delay_ms(DELAY_LINE_MS);


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
        delay_ms(DELAY_LINE_MS);


\
        
        
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
	delay_ms(DELAY_LINE_MS*2);
	LCD_sendbyte(LCD_RESET_ADDR,CMD_RS);
	delay_ms(DELAY_LINE_MS*2);
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
  //uint8_t addrToMatch;
  //do
  //{
  	LCD_sendbyte(ADDR,CMD_RS);
        delay_ms(DELAY_LINE_MS);
 
  //      addrToMatch  = LCD_WaitBusy();
  //      addrToMatch |= 0x80; //add extra bit so it matches input addr
  //}
 // while (ADDR != addrToMatch);
        
        
  
}


void LCD_CursorOn()
{
	LCD_sendbyte(LCD_LOWER_ADDR,CMD_RS);
        delay_ms(DELAY_LINE_MS*2);

	LCD_sendbyte (0x0E, CMD_RS);
	delay_ms(DELAY_LINE_MS*2);

}


void LCD_CursorOff()
{
	LCD_sendbyte(LCD_LOWER_ADDR,CMD_RS);
        delay_ms(DELAY_LINE_MS*2);

	LCD_sendbyte (0x0C, CMD_RS);
	delay_ms(DELAY_LINE_MS*2);

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






















