/**
 * STM32F103C8 Parking Slot
 */
#include "tools.h"
#include "nvic.h"
#include "conf.h"
#include "RFID.h"
/* Exported constants --------------------------------------------------------*/
void rfid_check(uint8_t *AuthedID);
void bell(uint16_t hz);

uint8_t AuthedID[5] = {148,37,132,99,86};
uint8_t isfull = 0;
int main(void)
{
  SystemInit();
  UserInit();
  PWM_Init();

  TM_MFRC522_Init();

  TIM3->CNT = 0;
  myprintf("----CHIP STARTED---\r\n");
  TIM4->CCR2=0;
  TIM4->CCR1=20000;
  uint8_t parking_state = 0;
  while(1){
    if (!GPIO_ReadInputDataBit(GPIOA, TRIG1_Pin)) {
      parking_state |= 1;
    } else {
      parking_state &= 6;
    } 
    if (!GPIO_ReadInputDataBit(GPIOA, TRIG2_Pin)) {
      parking_state |= 2;
    } else {
      parking_state &= 5;
    }
    if (!GPIO_ReadInputDataBit(GPIOA, TRIG3_Pin)) {
      parking_state |= 4;
    } else {
      parking_state &= 3;
    }
    isfull = (parking_state == 7) ? 1 : 0;
    if (isfull == 1) {
      GPIO_SetBits(GPIOB, GPIO_Pin_0);
    } else {
      GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    }
    rfid_check(AuthedID);

  }
  return 0;
}

void rfid_check(uint8_t *AuthedID)
{
  uint8_t CardID[5] = {0};
  if (TM_MFRC522_Check(CardID) == MI_OK) {
    myprintf("Found Card!\r\n");
    if (isfull == 0)
    {
      if (TM_MFRC522_Compare(CardID, AuthedID) == MI_OK) {
        myprintf("Authorized Card: id");
        myprintf("[%d-%d-%d-%d-%d]\r\n", 
            CardID[0], CardID[1], CardID[2], CardID[3], CardID[4]);
        myprintf("Open The Gate\r\n");
        TIM4->CCR2=2050;
        delay_ms(20000);
        TIM4->CCR2=950;
      } else {
        myprintf("Unauthorized Card: id");
        myprintf("[%d-%d-%d-%d-%d]\r\n", 
            CardID[0], CardID[1], CardID[2], CardID[3], CardID[4]);
        bell(15000);
      }

    } else {
      bell(18000);
    }
  }
  delay_ms(10);
}
void bell(uint16_t hz)
{

  TIM4->CCR1=20000-hz;
  delay_ms(20000);
  TIM4->CCR1=20000;
}

void rfid_auth(uint8_t *CardID)
{
  /*TM_MFRC522_Auth(PICC_AUTHENT1A, u8 BlockAddr, u8* Sectorkey, u8* serNum)*/
}
