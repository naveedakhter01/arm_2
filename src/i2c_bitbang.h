

#ifndef __I2C_BITBANG_H
#define __I2C_BITBANG_H



#include "stm32f10x.h"


#define I2C1_SCL_PIN         GPIO_Pin_10
#define I2C1_SDA_PIN         GPIO_Pin_11
#define I2C1_PORT            GPIOB
#define I2C1_CLK             RCC_APB2Periph_GPIOB

#define I2C2_SCL_PIN         GPIO_Pin_8
#define I2C2_SDA_PIN         GPIO_Pin_9
#define I2C2_PORT            GPIOA
#define I2C2_CLK             RCC_APB2Periph_GPIOA


#define ACK       0x00
#define NACK      0x80

//I2C1
void i2c1_init(void);
void i2c1_start(void);
void i2c1_stop(void);
void i2c1_bit_out(uint8_t data);
void i2c1_bit_in(uint8_t *data);
uint8_t i2c1_wr(uint8_t data);
uint8_t i2c1_rd(uint8_t ack);
void i2c1_ack_poll (uint8_t cntrl);


//I2C2
void i2c2_init(void);
void i2c2_start(void);
void i2c2_stop(void);
void i2c2_bit_out(uint8_t data);
void i2c2_bit_in(uint8_t *data);
uint8_t i2c2_wr(uint8_t data);
uint8_t i2c2_rd(uint8_t ack);
void i2c2_ack_poll (uint8_t cntrl);



void WaitForI2c2(void);








#endif //__I2C_BITBANG_H