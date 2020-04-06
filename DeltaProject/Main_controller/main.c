#include "main.h"


#define BUFF_SIZE_RX 18
#define BUFF_SIZE_TX 1
uint8_t rxbuff[BUFF_SIZE_RX] ;
uint8_t txbuff[BUFF_SIZE_TX] ;

#define BUFF_SIZE_TX_EVAL 4
uint8_t txbuff_AGV[BUFF_SIZE_TX_EVAL] ;
extern int8_t Position1, Position2, Position3;
extern double Position_SP1, Position_SP2, Position_SP3;

uint8_t set_home_complete[3] = {0,0,0};

void delay(uint32_t tui)
{
	while(tui--);
}

int main(void)
{
	RCC_configuration();
	Config_Proximity();
	USART_DMA_Config(115200);
	USART_EVAL_Config(38400);
	Config_send_driver();
	Config_send_PC();
	Config_IO();
	CAN_Config();	
	delay(300000);
	CanWriteData(0, mode_sethome);
//	SysTick_Config(SystemCoreClock/100);//10s
	
	while(1)
	{
		if(set_home_complete[0] == 1 && set_home_complete[1] == 1 && set_home_complete[2] == 1)
		{
			CanWriteData(40.0, mode_send_position1);
			CanWriteData(40.0, mode_send_position2);
			CanWriteData(40.0, mode_send_position3);
			USART_Cmd(USART3, ENABLE);
			break;
		}
	}
	while(1)
	{
	}
	return 0;
}

//void SysTick_Handler(void)
//{
//static uint16_t tui = 0, tui1 =0;
//	if(tui > 500)
//	{
//		tui =0;
//		if(tui1 % 2==0)
//		{
//				txbuff_AGV[0] = 'r'; txbuff_AGV[1] = 'u'; txbuff_AGV[2] = 'n'; txbuff_AGV[3] = '^';
//		}
//		else {txbuff_AGV[0] = 's'; txbuff_AGV[1] = 't'; txbuff_AGV[2] = 'o'; txbuff_AGV[3] = '^';}
//		tui1++;
//		DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
//		DMA1_Stream4->NDTR = 4;
//		DMA_Cmd(DMA1_Stream4, ENABLE);
//	}
//	else tui ++;
//	
//}

void Send_data_eval(void)
{
	uint8_t Position_SP1_temp = 0;
	if(Position_SP1 < 0)
	{
		txbuff_AGV[0] = '1';
		Position_SP1_temp = (uint8_t)(-Position_SP1);
	}
	else
	{
		txbuff_AGV[0] = '0';
		Position_SP1_temp = (uint8_t)(Position_SP1);		
	}
	txbuff_AGV[1] = (uint8_t)(Position_SP1_temp/10)+48;
	txbuff_AGV[2] = (uint8_t)(Position_SP1_temp%10)+48;
//////////////////
	uint8_t Position_SP2_temp = 0;	
	if(Position_SP2 < 0)
	{
		txbuff_AGV[3] = '1';
		Position_SP2_temp = (uint8_t)(-Position_SP2);
	}
	else
	{
		txbuff_AGV[3] = '0';
		Position_SP2_temp = (uint8_t)(Position_SP2);		
	}
	txbuff_AGV[4] = (uint8_t)(Position_SP2_temp/10)+48;
	txbuff_AGV[5] = (uint8_t)(Position_SP2_temp%10)+48;
/////////////////////
	uint8_t Position_SP3_temp = 0;	
	if(Position_SP3 < 0)
	{
		txbuff_AGV[6] = '1';
		Position_SP3_temp = (uint8_t)(-Position_SP3);
	}
	else
	{
		txbuff_AGV[6] = '0';
		Position_SP3_temp = (uint8_t)(Position_SP3);		
	}
	txbuff_AGV[7] = (uint8_t)(Position_SP3_temp/10)+48;
	txbuff_AGV[8] = (uint8_t)(Position_SP3_temp%10)+48;
//////////////////
	uint8_t Position1_temp = 0;	
	if(Position1 < 0)
	{
		txbuff_AGV[9] = '1';
		Position1_temp = (uint8_t)(-Position1);
	}
	else
	{
		txbuff_AGV[9] = '0';
		Position1_temp = (uint8_t)(Position1);		
	}
	txbuff_AGV[10] = (uint8_t)(Position1_temp/10)+48;
	txbuff_AGV[11] = (uint8_t)(Position1_temp%10)+48;
////////////////
	uint8_t Position2_temp = 0;	
	if(Position2 < 0)
	{
		txbuff_AGV[12] = '1';
		Position2_temp = (uint8_t)(-Position2);
	}
	else
	{
		txbuff_AGV[12] = '0';
		Position2_temp = (uint8_t)(Position2);		
	}
	txbuff_AGV[13] = (uint8_t)(Position2_temp/10)+48;
	txbuff_AGV[14] = (uint8_t)(Position2_temp%10)+48;
///////////////
	uint8_t Position3_temp = 0;	
	if(Position3 < 0)
	{
		txbuff_AGV[15] = '1';
		Position3_temp = (uint8_t)(-Position3);
	}
	else
	{
		txbuff_AGV[15] = '0';
		Position3_temp = (uint8_t)(Position3);		
	}
	txbuff_AGV[16] = (uint8_t)(Position3_temp/10)+48;
	txbuff_AGV[17] = (uint8_t)(Position3_temp%10)+48;

	
	DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
	DMA1_Stream4->NDTR = BUFF_SIZE_TX_EVAL;
	DMA_Cmd(DMA1_Stream4, ENABLE);
}

void RCC_configuration (void)
{
// Resets the clock configuration to the default reset state
RCC_DeInit();
// Enable external crystal (HSE)
RCC_HSEConfig(RCC_HSE_ON);
// Wait until HSE ready to use or not
ErrorStatus errorStatus = RCC_WaitForHSEStartUp();
if (errorStatus == SUCCESS)
{	
// Configure the PLL for 168MHz SysClk and 48MHz for USB OTG, SDIO
RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);
// Enable PLL
RCC_PLLCmd(ENABLE);
// Wait until main PLL clock ready
while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
// AHB 168MHz
RCC_HCLKConfig(RCC_SYSCLK_Div1);
// APB1 42MHz
RCC_PCLK1Config(RCC_HCLK_Div4);
// APB2 84 MHz
RCC_PCLK2Config(RCC_HCLK_Div2);
// Set SysClk using PLL
RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
}
else
{
// Do something to indicate that error clock configuration
while (1);
}
}

void Config_IO(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// Config IO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
	GPIO_SetBits(GPIOD, GPIO_Pin_12);
}


void Config_Proximity(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1);
	EXTI_InitStructure.EXTI_Line = EXTI_Line1; 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStructure);
		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_Init(&NVIC_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line1);
	
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource2);
	EXTI_InitStructure.EXTI_Line = EXTI_Line2; 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStructure);
		NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_Init(&NVIC_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line2);
	
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);
	EXTI_InitStructure.EXTI_Line = EXTI_Line3; 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStructure);
		NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	NVIC_Init(&NVIC_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line3);
	
}


void Config_send_PC(void)//////0.3s
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;
	TIM_TimeBaseStructure.TIM_Period = 3000 - 1;		
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2,DISABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	/* Enable the TIM2 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void Config_send_driver(void)//////0.01s
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;
	TIM_TimeBaseStructure.TIM_Period = 100 - 1;		
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM7,DISABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	/* Enable the TIM2 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}



void USART_DMA_Config(unsigned int BaudRate)
{
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		// Cogfig AF
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
	// PA0 -> Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	// PA1 -> Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// Config uart
	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	
	/* Enable USART */
	USART_Cmd(USART3, DISABLE);
	/* Enable USART4 DMA */
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	/* DMA1 Stream1 Channel1 for USART3 Rx configuration */
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxbuff;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = BUFF_SIZE_RX;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream1, &DMA_InitStructure);	
	DMA_Cmd(DMA1_Stream1, ENABLE);
	
	/* DMA1 Stream3 Channel4 for USART3 Tx configuration */
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)txbuff;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = BUFF_SIZE_TX;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream3, ENABLE);
	
	/* Enable DMA Interrupt to the highest priority */
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* Transfer complete interrupt mask */
	DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);
}

void USART_EVAL_Config(unsigned int BaudRate)
{
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		// Cogfig AF
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
	// PC10 -> Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// Config uart
	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);
	
	/* Enable UART */
	USART_Cmd(UART4, ENABLE);
	/* Enable UART4 DMA */
	USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);

	
	/* DMA1 Stream3 Channel4 for UART4 Tx configuration */
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)txbuff_AGV;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = BUFF_SIZE_TX_EVAL;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream4, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream4, ENABLE);
}