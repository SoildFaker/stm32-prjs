// 系统中断
#include "conf.h"

u16 timer_counter;

/******************************************************************************
* 处理器异常中断列表
*******************************************************************************/

void NMI_Handler(void)
{
  // 本中断在HSE时钟启动失败时触发

  if (RCC_GetITStatus(RCC_IT_CSS) != RESET) {
    /* At this stage: HSE, PLL are disabled (but no change on PLL config) and HSI
       is selected as system clock source */

    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Enable HSE Ready interrupt */
    RCC_ITConfig(RCC_IT_HSERDY, ENABLE);

#ifndef SYSCLK_HSE
 #ifdef STM32F10X_CL
    /* Enable PLL and PLL2 Ready interrupts */
    RCC_ITConfig(RCC_IT_PLLRDY | RCC_IT_PLL2RDY, ENABLE);
 #else
    /* Enable PLL Ready interrupt */
    RCC_ITConfig(RCC_IT_PLLRDY, ENABLE);
 #endif	/* STM32F10X_CL */
#endif /* SYSCLK_HSE */

    /* Clear Clock Security System interrupt pending bit */
    RCC_ClearITPendingBit(RCC_IT_CSS);

    /* Once HSE clock recover, the HSERDY interrupt is generated and in the RCC ISR
       routine the system clock will be reconfigured to its previous state (before
       HSE clock failure) */
  }
}

void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1) {
  }
}

void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1) {
  }
}

void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1) {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
}
/******************************************************************************
* 外部设备中断表
*******************************************************************************/
void TIM2_IRQHandler(void)
{
  //检测是否发生溢出更新事件
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    //清除TIM2的中断待处理位
    TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);
    timer_counter++;
  }
}

void RCC_IRQHandler(void)
{
  if(RCC_GetITStatus(RCC_IT_HSERDY) != RESET)
  { 
    /* Clear HSERDY interrupt pending bit */
    RCC_ClearITPendingBit(RCC_IT_HSERDY);

    /* Check if the HSE clock is still available */
    if (RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET)
    { 
#ifdef SYSCLK_HSE
      /* Select HSE as system clock source */
      RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);
#else
 #ifdef STM32F10X_CL
      /* Enable PLL2 */
      RCC_PLL2Cmd(ENABLE);
 #else
      /* Enable PLL: once the PLL is ready the PLLRDY interrupt is generated */ 
      RCC_PLLCmd(ENABLE);
 #endif	/* STM32F10X_CL */
#endif /* SYSCLK_HSE */      
    }
  }

#ifdef STM32F10X_CL
  if(RCC_GetITStatus(RCC_IT_PLL2RDY) != RESET)
  { 
    /* Clear PLL2RDY interrupt pending bit */
    RCC_ClearITPendingBit(RCC_IT_PLL2RDY);

    /* Enable PLL: once the PLL is ready the PLLRDY interrupt is generated */ 
    RCC_PLLCmd(ENABLE);
  }
#endif /* STM32F10X_CL */   

  if(RCC_GetITStatus(RCC_IT_PLLRDY) != RESET)
  { 
    /* Clear PLLRDY interrupt pending bit */
    RCC_ClearITPendingBit(RCC_IT_PLLRDY);

    /* Check if the PLL is still locked */
    if (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != RESET)
    { 
      /* Select PLL as system clock source */
      RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    }
  }
}
