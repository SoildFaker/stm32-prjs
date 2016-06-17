// 系统硬件控制

/* Includes ------------------------------------------------------------------*/
#include "../include/STM32/misc.h"
#include "../src/conf.h"


#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

// 系统中断控制器 
// 分组抢占优先级 
void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup)
{
  // 检查参数
  assert_param(IS_NVIC_PRIORITY_GROUP(NVIC_PriorityGroup));
  
  // 根据参数设置组优先级寄存器中的10-8位
  SCB->AIRCR = AIRCR_VECTKEY_MASK | NVIC_PriorityGroup;
}

// 系统中断控制器初始化
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct)
{
  uint32_t tmppriority = 0x00, tmppre = 0x00, tmpsub = 0x0F;
  
  // 检查参数
  assert_param(IS_FUNCTIONAL_STATE(NVIC_InitStruct->NVIC_IRQChannelCmd));
  assert_param(IS_NVIC_PREEMPTION_PRIORITY(NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority));  
  assert_param(IS_NVIC_SUB_PRIORITY(NVIC_InitStruct->NVIC_IRQChannelSubPriority));
    
  if (NVIC_InitStruct->NVIC_IRQChannelCmd != DISABLE) {
    // 计算 IRQ 访问优先级
    tmppriority = (0x700 - ((SCB->AIRCR) & (uint32_t)0x700))>> 0x08;
    tmppre = (0x4 - tmppriority);
    tmpsub = tmpsub >> tmppriority;

    tmppriority = (uint32_t)NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority << tmppre;
    tmppriority |=  NVIC_InitStruct->NVIC_IRQChannelSubPriority & tmpsub;
    tmppriority = tmppriority << 0x04;
        
    NVIC->IP[NVIC_InitStruct->NVIC_IRQChannel] = tmppriority;
    
    // 使能IRQ通道
    NVIC->ISER[NVIC_InitStruct->NVIC_IRQChannel >> 0x05] =
      (uint32_t)0x01 << (NVIC_InitStruct->NVIC_IRQChannel & (uint8_t)0x1F);
  } else {
    // 关闭IRQ通道
    NVIC->ICER[NVIC_InitStruct->NVIC_IRQChannel >> 0x05] =
      (uint32_t)0x01 << (NVIC_InitStruct->NVIC_IRQChannel & (uint8_t)0x1F);
  }
}

// 设置中断向量表位置
// 参数1 定义向量表的位置NVIC_VectTab_RAM或者NVIC_VectTab_FLASH
// 参数2 中断向量表起始偏移 这个值会乘以0x200
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset)
{ 
  // 检查参数
  assert_param(IS_NVIC_VECTTAB(NVIC_VectTab));
  assert_param(IS_NVIC_OFFSET(Offset));  
   
  SCB->VTOR = NVIC_VectTab | (Offset & (uint32_t)0x1FFFFF80);
}

// 系统进入节能模式的条件
// 参数1 选择不同节能模式NVIC_LP_SEVONPEND NVIC_LP_SLEEPDEEP NVIC_LP_SLEEPONEXIT
// 参数2 ENABLE DISABLE.
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState)
{
  // 检查参数
  assert_param(IS_NVIC_LP(LowPowerMode));
  assert_param(IS_FUNCTIONAL_STATE(NewState));  
  
  if (NewState != DISABLE) {
    SCB->SCR |= LowPowerMode;
  } else {
    SCB->SCR &= (uint32_t)(~(uint32_t)LowPowerMode);
  }
}

// 设置系统定时器的信号源
// 参数 一般是HCLK时钟 SysTick_CLKSource_HCLK_Div8  SysTick_CLKSource_HCLK 
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource)
{
  // 检查参数
  assert_param(IS_SYSTICK_CLK_SOURCE(SysTick_CLKSource));
  if (SysTick_CLKSource == SysTick_CLKSource_HCLK) {
    SysTick->CTRL |= SysTick_CLKSource_HCLK;
  } else {
    SysTick->CTRL &= SysTick_CLKSource_HCLK_Div8;
  }
}
