#include "conf.h"
#include "tools.h"
#include "magnet.h"

uint16_t xsdat,ysdat,zsdat;

void MAG3110_Init()
{
  uint8_t temp;
  I2C_BufferRead(MAG_Addr,&temp,WHO_AM_I_REG ,1);
  myprintf("The WHO AM I of MAG3110 is : 0x%x\r\n",temp); //test!
  if(temp == 0xc4)
    myprintf("The ID is OK!\n\r");
  else
    myprintf("The ID is Wrong!\n\r");

  delay_ms(100);
  I2C_ByteWrite(MAG_Addr,0x80,CTRL_REG2); //Automatic Magnetic Sensor Reset
  delay_ms(100);
  I2C_ByteWrite(MAG_Addr,0x12,CTRL_REG1); //Trigger immediate with 20 Hz and then return to standby mode
  delay_ms(100);
}

void Data_Manage(uint8_t *BUF)
{
  xsdat = ((BUF[0]<<8) | BUF[1]) - 0x0800;//Minus the offset determined according to the practical situation
  ysdat = ((BUF[2]<<8) | BUF[3]) - 0xfa00;
  zsdat = (BUF[4]<<8) | BUF[5];
}

void I2C_ByteWrite(u8 DevAddr,u8 WData, u8 WriteAddr)
{
  /* Send STRAT condition */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  

  /* Send address for write */
  I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Transmitter);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
      
  /* Send the EEPROM's internal address to write to */
  I2C_SendData(I2C1, WriteAddr);
  
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(I2C1, WData); 
   
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send STOP condition */
  I2C_GenerateSTOP(I2C1, ENABLE);
}

void I2C_BufferRead(u8 DevAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{  
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  /* Send START condition */
  I2C_GenerateSTART(I2C1, ENABLE);
  //*((u8 *)0x4001080c) &=~0x80;
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  I2C_Send7bitAddress(I2C1,DevAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(I2C1, ENABLE);

  I2C_SendData(I2C1, ReadAddr);  

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send STRAT condition a second time */  
  I2C_GenerateSTART(I2C1, ENABLE);
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  
  I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Receiver);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  
  /* While there is data to be read */
  while(NumByteToRead)  
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2C1, DISABLE);
      
      /* Send STOP Condition */
      I2C_GenerateSTOP(I2C1, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))  
    {      
      /* Read a byte from the EEPROM */
      *pBuffer = I2C_ReceiveData(I2C1);

      /* Point to the next location where the byte read will be saved */
      pBuffer++; 
      
      /* Decrement the read bytes counter */
      NumByteToRead--;        
    }   
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2C1, ENABLE);
}
