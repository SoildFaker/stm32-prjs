#include "conf.h"


void RCC_Conf(void)
{
  ErrorStatus HSEStartUpStatus;
  //复位RCC外部设备寄存器到默认值
  RCC_DeInit();  
  //打开外部高速晶振
  RCC_HSEConfig(RCC_HSE_ON); 
  //等待外部高速时钟准备好
  HSEStartUpStatus = RCC_WaitForHSEStartUp(); 
  if(HSEStartUpStatus == SUCCESS){
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency(FLASH_Latency_2);
    //配置AHB(HCLK)时钟=SYSCLK
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    //配置APB2(PCLK2)钟=AHB时钟
    RCC_PCLK2Config(RCC_HCLK_Div1); 
    //配置APB1(PCLK1)钟=AHB 1/2时钟
    RCC_PCLK1Config(RCC_HCLK_Div2);  
    //配置ADC时钟=PCLK2 1/4
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
    //配置PLL时钟 == 外部高速晶体时钟*9
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); 
    //配置ADC时钟= PCLK2/4
    RCC_ADCCLKConfig(RCC_PCLK2_Div4);
    //使能PLL时钟
    RCC_PLLCmd(ENABLE);  
    //等待PLL时钟就绪
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET); //配置系统时钟 = PLL时钟
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); 
    //检查PLL时钟是否作为系统时钟
    while(RCC_GetSYSCLKSource() != 0x08);
  }
  
  //开启AFIO时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  //SPI
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);	//SPI时钟
	//Enable clock source for i2c
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  //Enable GPIO timer
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  //Enable serial timer
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  //Timer
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //配置RCC，使能TIM2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //配置RCC，使能TIM3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
}

// 串口相关配置
void USART_Conf(void)
{
  USART_InitTypeDef USART_InitStructure;
  //串口配置： 波特率 115200 数据位 8 停止位 1  奇偶位 NONE  
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity =  USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  //初始化串口
  USART_Init(USART2, &USART_InitStructure);
  //启动串口
  USART_Cmd(USART2, ENABLE);
}

// I2C通讯的配置，主要用来和陀螺仪芯片通讯
void I2C_Conf(void)
{
	I2C_InitTypeDef I2C_InitStructure;
	
	//Configuration I2C
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_OwnAddress1 = 0x01;
	I2C_Init(MPU_I2Cx,&I2C_InitStructure);
	I2C_Cmd(MPU_I2Cx,ENABLE);
}
// 定时器设置 
void TIMER_Conf(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  
  TIM_TimeBaseStructure.TIM_Prescaler = 71;  //时钟预分频数 例如:时钟频率=72/(时钟预分频+1)  
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; ////定时器模式 向上计数  
  TIM_TimeBaseStructure.TIM_Period = 0xffff;//自动重装载寄存器周期的值(定时时间)累计 0xFFFF个频率后产生个更新或者中断(也是说定时时间到)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; ////时间分割值  
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

/******************************************************************************/
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //初始化定时器2
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //打开中断 溢出中断  
  TIM_Cmd(TIM2, ENABLE);// TIM2 enable counter [允许tim2计数]

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //初始化定时器3
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //打开中断 溢出中断  
  TIM_Cmd(TIM3, ENABLE);//主更新
}

void SPI_Conf(void)
{
  SPI_InitTypeDef   SPI_InitStructure;

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //设置SPI为双线双向全双工模式
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	 //主机模式
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//发送、接收8位帧结构
  SPI_InitStructure.SPI_CPOL =SPI_CPOL_High ; //始终悬空高  // SPI_CPOL_Low//始终悬空低 
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//第2个时钟沿捕获 //SPI_CPHA_1Edge第1个时钟沿捕获 
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	 //硬件控制NSS信号（ss） 置成软件时,NSS脚可以他用	  // SPI_NSS_Hard 
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;  //预分频值为64
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //数据传输由最高位开始	   //SD卡高位先传送
  SPI_InitStructure.SPI_CRCPolynomial = 7;	 //定义了CRC值计算的多项式为7
  SPI_Init(SPI1, &SPI_InitStructure); 
  SPI_Cmd(SPI1,ENABLE); 
}

void PWM_Conf(void)
{
	TIM_TimeBaseInitTypeDef TM4_TB_Config;
	TIM_OCInitTypeDef TM4_OC_Config;
	
	//first, setup timebase structure
	TM4_TB_Config.TIM_ClockDivision=TIM_CKD_DIV1 ;
	TM4_TB_Config.TIM_CounterMode=TIM_CounterMode_Up;
	TM4_TB_Config.TIM_Period=5000;//T=Period*t_step=2.5ms
	TM4_TB_Config.TIM_Prescaler=36-1;//t_step=(Prescaler+1)/core_clock=0.5us
	TIM_TimeBaseInit(TIM4,&TM4_TB_Config);
	//second, setup output compare structure
	TM4_OC_Config.TIM_OCMode=TIM_OCMode_PWM1;
	TM4_OC_Config.TIM_OCPolarity=TIM_OCPolarity_High;
	TM4_OC_Config.TIM_OutputState=TIM_OutputState_Enable ;
	TM4_OC_Config.TIM_Pulse=0;
	TIM_OC1Init(TIM4,&TM4_OC_Config);
	TIM_OC2Init(TIM4,&TM4_OC_Config);
	TIM_OC3Init(TIM4,&TM4_OC_Config);
	TIM_OC4Init(TIM4,&TM4_OC_Config);
	
	//enable auto reload for OC_reg and ARR_reg
	TIM_ARRPreloadConfig(TIM4,ENABLE);//can change T 
	TIM_OC1PreloadConfig(TIM4,ENABLE);//can change t_on
	TIM_OC2PreloadConfig(TIM4,ENABLE);//can change t_on
	TIM_OC3PreloadConfig(TIM4,ENABLE);//can change t_on
	TIM_OC4PreloadConfig(TIM4,ENABLE);//can change t_on
	TIM_Cmd(TIM4,ENABLE);
}


void GPIO_Conf(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  //USART
  GPIO_InitStructure.GPIO_Pin = GPIO_TxPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//USARTx_Tx为复合推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_RxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//USARTx_Rx 为浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //HC-SR04 
  GPIO_InitStructure.GPIO_Pin = TRIG_Pin; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = ECHO_Pin; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	//MPU6050-IIC
	GPIO_InitStructure.GPIO_Pin = SCL_Pin | SDA_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  //PWM Motor
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin=CW1_Pin|CW2_Pin|CCW1_Pin|CCW2_Pin;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

  //SPI Optical flow
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP; //推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP; //推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 系统中断控制
void NVIC_Conf(void)
{ 
  NVIC_InitTypeDef NVIC_InitStructure;
  // 设置中断向量表位置 FLASH:0x08000000
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); 

  // 打开TIM2中断
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


