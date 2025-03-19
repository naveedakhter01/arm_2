


#include "i2c_bitbang.h"

#define DELAY1 3
#define DELAY2 5



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// I2C1
//



/*******************************************************************************
* Function Name  : I2C_BITBANG_+Init
* Description    : Initializes the I2C.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void i2c1_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* GPIOB Periph clock enable */
  RCC_APB2PeriphClockCmd( I2C1_CLK, ENABLE);

  /* Configure I2C pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  I2C1_SCL_PIN | I2C1_SDA_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(I2C1_PORT, &GPIO_InitStructure);

  I2C1_PORT->BSRR = I2C1_SDA_PIN;  //sda is high
  I2C1_PORT->BSRR = I2C1_SCL_PIN;  //scl is high
}


//....................................................................
// This function generates an I2C Start Condition
//....................................................................
void i2c1_start(void)
{
unsigned int i;
  I2C1_PORT->BSRR = I2C1_SDA_PIN;  //ensure sda is high
  for (i=0;i<DELAY1;i++)  {   __no_operation(); }               //delay
  
  I2C1_PORT->BSRR = I2C1_SCL_PIN;  //ensure scl is high 
  for (i=0;i<DELAY2;i++)  {   __no_operation(); }               //delay
  
  I2C1_PORT->BRR = I2C1_SDA_PIN;  //pull sda low
  for (i=0;i<DELAY2;i++)  {   __no_operation(); }               //delay
  
  I2C1_PORT->BRR = I2C1_SCL_PIN;  //pull scl is low 
  for (i=0;i<DELAY2;i++)  {   __no_operation(); }               //delay
}

//....................................................................
// This function generates an I2C Stop Condition
//....................................................................
void i2c1_stop(void)
{
unsigned int i;

  I2C1_PORT->BRR = I2C1_SCL_PIN;  //ensure scl is low 
  I2C1_PORT->BRR = I2C1_SDA_PIN;  //pull sda low
  for (i=0;i<DELAY2;i++)  {   __no_operation(); }               //delay
 
  I2C1_PORT->BSRR = I2C1_SCL_PIN;  // pull scl high 
  for (i=0;i<DELAY2;i++)  {   __no_operation(); }               //delay
  
  I2C1_PORT->BSRR = I2C1_SDA_PIN;  //pull sda high SDA_TRIS = 1
  for (i=0;i<DELAY2;i++)  {   __no_operation(); }               //delay
}


//....................................................................
// Outputs a bit to the I2C bus
//....................................................................
void i2c1_bit_out(uint8_t data)
{
  uint8_t i;
  
  I2C1_PORT->BRR = I2C1_SCL_PIN;  //ensure scl is low 
  
  if(data>>7) { I2C1_PORT->BSRR = I2C1_SDA_PIN; }  //high
  else { I2C1_PORT->BRR = I2C1_SDA_PIN; }  //low
  for (i=0;i<DELAY1;i++)  {   __no_operation(); }               //delay
  
  I2C1_PORT->BSRR = I2C1_SCL_PIN;  //SCL high to clock bit
  for (i=0;i<DELAY2;i++)  {   __no_operation(); }               //delay
  
  I2C1_PORT->BRR = I2C1_SCL_PIN;                    // pull SCL low for next bit
  for (i=0;i<DELAY1;i++)  {   __no_operation(); }               //delay
}


//....................................................................
// Inputs a bit from the I2C bus
//....................................................................
void i2c1_bit_in(uint8_t *data)
{
  uint8_t i;

  I2C1_PORT->BRR = I2C1_SCL_PIN;  //ensure scl is low 
  for (i=0;i<DELAY1;i++)  {   __no_operation(); }               //delay
  
  I2C1_PORT->BSRR = I2C1_SDA_PIN;  //make SDA high so i2c device can pull it low
  for (i=0;i<DELAY1;i++)  {   __no_operation(); }               //delay
  
  I2C1_PORT->BSRR = I2C1_SCL_PIN;                      // bring SCL high to begin transfer
  for (i=0;i<DELAY2;i++)  {   __no_operation(); }               //delay
  
  //read sda bit in
  if ((I2C1_PORT->IDR & I2C1_SDA_PIN) != (uint32_t)Bit_RESET)
  {
    *data |= 0x01;
  }
  else
  {
    *data |= 0x00;
  }
  I2C1_PORT->BRR = I2C1_SCL_PIN;                        // bring SCL low again.
  for (i=0;i<DELAY1;i++)  {   __no_operation(); }               //delay
}


//....................................................................
// Writes a byte to the I2C bus
//....................................................................
uint8_t i2c1_wr(uint8_t data)
{
  uint8_t i;                // loop counter
  uint8_t ack;              // ACK bit

  ack = 0;
  for (i = 0; i < 8; i++)         // loop through each bit
      {
      i2c1_bit_out(data);              // output bit
      data = data << 1;           // shift left for next bit
      }

  i2c1_bit_in(&ack);                   // input ACK bit
  return ack;
}


//....................................................................
// Reads a byte from the I2C bus
//....................................................................
uint8_t i2c1_rd(uint8_t ack)
{
  uint8_t i;                // loop counter
  uint8_t ret=0;            // return value

  for (i = 0; i < 8; i++)         // loop through each bit
      {
      ret = ret << 1;             // shift left for next bit
      i2c1_bit_in(&ret);               // input bit
      }
  
  i2c1_bit_out(ack);                   // output ACK/NAK bit
  return ret;
}


//.............................................................................
//          Polls the bus for ACK from device
//.............................................................................
void i2c1_ack_poll (uint8_t cntrl)
{
  uint8_t result=1;
  
  while(result)
          {
          i2c1_start();            // generate Restart condition
          result=i2c1_wr(cntrl); // send control byte (WRITE command)
          }
  i2c1_stop();                     // generate Stop condition
}













////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// I2C2
//



/*******************************************************************************
* Function Name  : I2C_BITBANG_+Init
* Description    : Initializes the I2C.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void i2c2_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* GPIOB Periph clock enable */
  RCC_APB2PeriphClockCmd( I2C2_CLK, ENABLE);

  /* Configure I2C pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  I2C2_SCL_PIN | I2C2_SDA_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(I2C2_PORT, &GPIO_InitStructure);

  I2C2_PORT->BSRR = I2C2_SDA_PIN;  //sda is high
  I2C2_PORT->BSRR = I2C2_SCL_PIN;  //scl is high
}


//....................................................................
// This function generates an I2C Start Condition
//....................................................................
void i2c2_start(void)
{
unsigned int i;
  I2C2_PORT->BSRR = I2C2_SDA_PIN;  //ensure sda is high
  for (i=0;i<DELAY1;i++)  {   __no_operation(); }               //delay
  
  I2C2_PORT->BSRR = I2C2_SCL_PIN;  //ensure scl is high 
  for (i=0;i<DELAY2;i++)  {   __no_operation(); }               //delay
  
  I2C2_PORT->BRR = I2C2_SDA_PIN;  //pull sda low
  for (i=0;i<DELAY2;i++)  {   __no_operation(); }               //delay
  
  I2C2_PORT->BRR = I2C2_SCL_PIN;  //pull scl is low 
  for (i=0;i<DELAY2;i++)  {   __no_operation(); }               //delay
}

//....................................................................
// This function generates an I2C Stop Condition
//....................................................................
void i2c2_stop(void)
{
unsigned int i;

  I2C2_PORT->BRR = I2C2_SCL_PIN;  //ensure scl is low 
  I2C2_PORT->BRR = I2C2_SDA_PIN;  //pull sda low
  for (i=0;i<DELAY2;i++)  {   __no_operation(); }               //delay
 
  I2C2_PORT->BSRR = I2C2_SCL_PIN;  // pull scl high 
  for (i=0;i<DELAY2;i++)  {   __no_operation(); }               //delay
  
  I2C2_PORT->BSRR = I2C2_SDA_PIN;  //pull sda high SDA_TRIS = 1
  for (i=0;i<DELAY2;i++)  {   __no_operation(); }               //delay
}


//....................................................................
// Outputs a bit to the I2C bus
//....................................................................
void i2c2_bit_out(uint8_t data)
{
  uint8_t i;
  
  I2C2_PORT->BRR = I2C2_SCL_PIN;  //ensure scl is low 
  
  if(data>>7) { I2C2_PORT->BSRR = I2C2_SDA_PIN; }  //high
  else { I2C2_PORT->BRR = I2C2_SDA_PIN; }  //low
  for (i=0;i<DELAY1;i++)  {   __no_operation(); }               //delay
  
  I2C2_PORT->BSRR = I2C2_SCL_PIN;  //SCL high to clock bit
  for (i=0;i<DELAY2;i++)  {   __no_operation(); }               //delay
  
  I2C2_PORT->BRR = I2C2_SCL_PIN;                    // pull SCL low for next bit
  for (i=0;i<DELAY1;i++)  {   __no_operation(); }               //delay
}


//....................................................................
// Inputs a bit from the I2C bus
//....................................................................
void i2c2_bit_in(uint8_t *data)
{
  uint8_t i;

  I2C2_PORT->BRR = I2C2_SCL_PIN;  //ensure scl is low 
  for (i=0;i<DELAY1;i++)  {   __no_operation(); }               //delay
  
  I2C2_PORT->BSRR = I2C2_SDA_PIN;  //make SDA high so i2c device can pull it low
  for (i=0;i<DELAY1;i++)  {   __no_operation(); }               //delay
  
  I2C2_PORT->BSRR = I2C2_SCL_PIN;                      // bring SCL high to begin transfer
  for (i=0;i<DELAY2;i++)  {   __no_operation(); }               //delay
  
  //read sda bit in
  if ((I2C2_PORT->IDR & I2C2_SDA_PIN) != (uint32_t)Bit_RESET)
  {
    *data |= 0x01;
  }
  else
  {
    *data |= 0x00;
  }
  I2C2_PORT->BRR = I2C2_SCL_PIN;                        // bring SCL low again.
  for (i=0;i<DELAY1;i++)  {   __no_operation(); }               //delay
}


//....................................................................
// Writes a byte to the I2C bus
//....................................................................
uint8_t i2c2_wr(uint8_t data)
{
  uint8_t i;                // loop counter
  uint8_t ack;              // ACK bit

  ack = 0;
  for (i = 0; i < 8; i++)         // loop through each bit
      {
      i2c2_bit_out(data);              // output bit
      data = data << 1;           // shift left for next bit
      }

  i2c2_bit_in(&ack);                   // input ACK bit
  return ack;
}


//....................................................................
// Reads a byte from the I2C bus
//....................................................................
uint8_t i2c2_rd(uint8_t ack)
{
  uint8_t i;                // loop counter
  uint8_t ret=0;            // return value

  for (i = 0; i < 8; i++)         // loop through each bit
      {
      ret = ret << 1;             // shift left for next bit
      i2c2_bit_in(&ret);               // input bit
      }
  
  i2c2_bit_out(ack);                   // output ACK/NAK bit
  return ret;
}


//.............................................................................
//          Polls the bus for ACK from device
//.............................................................................
void i2c2_ack_poll (uint8_t cntrl)
{
  uint8_t result=1;
  
  while(result)
          {
          i2c2_start();            // generate Restart condition
          result=i2c2_wr(cntrl); // send control byte (WRITE command)
          }
  i2c2_stop();                     // generate Stop condition
}


//
// waits untill i2c lines are free to use
// -- used while waiting for dsp to boot up from eeprom
void WaitForI2c2(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd( I2C2_CLK, ENABLE);

  /* Configure I2C pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  I2C2_SCL_PIN | I2C2_SDA_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(I2C2_PORT, &GPIO_InitStructure);
  
  int timeout = 10000;
  while(timeout-- > 0)
  {
     if ( (I2C2_PORT->IDR & I2C2_SDA_PIN) == (uint32_t)Bit_RESET ||
         (I2C2_PORT->IDR & I2C2_SCL_PIN) == (uint32_t)Bit_RESET)
        timeout = 1000;
  
     delay_us(1);
  }
}