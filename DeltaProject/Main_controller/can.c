#include "main.h"

uint8_t CAN_DATA_R[8] = {0};
uint16_t CAN_ID=0;
int8_t Position1 = 0, Position2 = 0, Position3 = 0;


void CanWriteData(double data,uint8_t mode)
{
	/*
	mode set_home --> ID = 0x100
	mode completa_home --> ID = 0x110
	mode set_normal ---> ID3 = 0x201 (motor1) , ID2 = 0x202 (motor2) , ID1 = 0x203 (motor3) ---> data = 5 byte
	mode test_accuracy --> ID3 = 0x301 (motor1) , ID2 = 0x302 (motor2) , ID1 = 0x303 (motor3) ---> data = 5 byte
	*/
	CanTxMsg TxMessage;
	  /* transmit */
	TxMessage.StdId = 0x000;
  TxMessage.ExtId = 0x01;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.IDE = CAN_ID_STD;
	uint16_t uwCounter = 0;
	uint8_t TransmitMailbox = 0;
	
	if(mode == mode_send_position1 | mode == mode_send_position2 | mode == mode_send_position3)
	{
		if(data<=60 && data>=-60)
		{
			if(mode == mode_send_position1) TxMessage.StdId = 0x201; 
			else if(mode == mode_send_position2) TxMessage.StdId = 0x202; 
			else if(mode == mode_send_position3) TxMessage.StdId = 0x203; 
			TxMessage.DLC = 5;
			if(data<0)
			{
				data = -data;
				TxMessage.Data[0] = 1;
			}
			else {TxMessage.Data[0] = 0;}
			if(data<10)
			{
				TxMessage.Data[1] = 0;
				data = 100*data;
			}
			else {TxMessage.Data[1] = 1; data = 10*data;}
			uint16_t data_char = data;
			TxMessage.Data[2] = data_char/100;
			TxMessage.Data[3] = (data_char%100)/10;
			TxMessage.Data[4] = (data_char%10);
			TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);
		}
	}
	else if(mode == mode_sethome)
	{
		TxMessage.DLC = 0;
		TxMessage.StdId = 0x100;
		TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);
	}
	else if(mode == mode_complete_home1)
	{
		TxMessage.DLC = 0;
		TxMessage.StdId = 0x111;
		TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);
	}
	else if(mode == mode_complete_home2)
	{
		TxMessage.DLC = 0;
		TxMessage.StdId = 0x112;
		TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);
	}
	else if(mode == mode_complete_home3)
	{
		TxMessage.DLC = 0;
		TxMessage.StdId = 0x113;
		TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);
	}
	else if(mode == mode_error)
	{
		TxMessage.DLC = 0;
		TxMessage.StdId = 0x010;
		TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);
	}
	while((CAN_TransmitStatus(CAN1, TransmitMailbox)  !=  CANTXOK) && (uwCounter  !=  0xFFFF))
  {
    uwCounter++;
  }
}

void CAN1_RX0_IRQHandler(void)
{
  CanRxMsg RxMessage;
  CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);
  CAN_ID=RxMessage.StdId;
	if(CAN_ID == 0x010)
	{
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		TIM_Cmd(TIM7,DISABLE);
		TIM_Cmd(TIM2,DISABLE);
		USART_Cmd(USART3, DISABLE);
	}
	else if(CAN_ID == 0x301)
	{
		Position1 = RxMessage.Data[1];
		if(RxMessage.Data[0] == 1) 
			Position1 = -Position1;
	}
	else if(CAN_ID == 0x302)
	{
		Position2 = RxMessage.Data[1];
		if(RxMessage.Data[0] == 1) 
			Position2 = -Position2;
	}
	else if(CAN_ID == 0x303)
	{
		Position3 = RxMessage.Data[1];
		if(RxMessage.Data[0] == 1) 
		Position3 = -Position3;
	}
	
  CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
}

void CAN_Config(void)
{
	//Configure GPIO
  /* CAN Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1);
  /* Configure CAN pin: RX */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOB, &GPIO_InitStructure);   
  /* Configure CAN pin: TX */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOB, &GPIO_InitStructure);	   

	
	//Configure CAN
  /* CAN register init */
  CAN_DeInit(CAN1);
	CAN_InitTypeDef CAN_InitStructure;
  CAN_StructInit(&CAN_InitStructure);
  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
  CAN_InitStructure.CAN_Prescaler = 14;
//// Baudrate = 500000KBps
  if (CAN_Init(CAN1,&CAN_InitStructure) == CANINITFAILED) 		
  {
  }	
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
  CAN_FilterInitStructure.CAN_FilterNumber=0;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);
  CAN_FilterInit(&CAN_FilterInitStructure);
	
	//Configure Interrupt : CAN1 RX0 interrupt IRQ channel
  /* Enable CAN1 RX0 interrupt IRQ channel */
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}