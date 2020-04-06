#include "main.h"

uint32_t CAN_ID=0;
extern double Set_Position, Position;
extern int32_t Rotary, Counter;

void USB_LP_CAN1_RX0_IRQHandler(void)
{
  CanRxMsg RxMessage;
  CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);
  CAN_ID=RxMessage.StdId;

	if(CAN_ID == 0x202) // receive position
	{
		TIM_Cmd(TIM2,ENABLE);
		double Set_Position_temp = 100*RxMessage.Data[2] + 10*RxMessage.Data[3] + RxMessage.Data[4];
		if(RxMessage.Data[0] == 1)
		{
			Set_Position_temp = -Set_Position_temp;
		}
		if(RxMessage.Data[1] == 0)
		{
			Set_Position_temp = Set_Position_temp/100;
		}
		else Set_Position_temp = Set_Position_temp/10;
		if(Set_Position_temp>=-60 && Set_Position_temp<=60)
		{Set_Position = Set_Position_temp;}
	//	CanWriteData(Position, mode_send_position);
	}	
	else if(CAN_ID == 0x100) // start set home
	{
		TIM_SetCompare1(TIM1,400);
		TIM_SetCompare2(TIM1,0);
	}
	else if(CAN_ID == 0x112) //set home complete
	{
		TIM_SetCompare1(TIM1,0);
		TIM_SetCompare2(TIM1,0);
		TIM_SetCounter(TIM3, 10000);
		Counter =0;		Rotary = 0;
	}
	else if(CAN_ID == 0x010)
	{
		TIM_Cmd(TIM2,DISABLE);
		TIM_SetCompare1(TIM1,0);
		TIM_SetCompare2(TIM1,0);				
		TIM_CtrlPWMOutputs(TIM1,DISABLE);			
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);			
	}
  CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
}

void CanWriteData(double data,uint8_t mode)
{

	CanTxMsg TxMessage;
	TxMessage.StdId = 0x000;
	if(mode == mode_error)
	{
		TxMessage.StdId = 0x010;
		TxMessage.DLC = 0;
	}
	else if(mode == mode_send_position)
	{
		TxMessage.StdId = 0x302;
		TxMessage.DLC = 2;
		if(data<0)
		{
			TxMessage.Data[0] = 1;
			data = -data;
		}
		else
		{
			TxMessage.Data[0] = 0;
		}
		TxMessage.Data[1] = (int8_t)data;
	}
	  /* transmit */
  TxMessage.ExtId = 0x01;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.IDE = CAN_ID_STD;
	CAN_Transmit(CAN1,&TxMessage);
}

void Can_config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* CAN Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE); 
  /* Configure CAN pin: RX */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);   
  /* Configure CAN pin: TX */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);	        

	
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;

  /* CAN register init */
  CAN_DeInit(CAN1);
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
  CAN_InitStructure.CAN_BS2 = CAN_BS1_2tq;
  CAN_InitStructure.CAN_Prescaler = 12;

  if (CAN_Init(CAN1,&CAN_InitStructure) == CANINITFAILED) 		
  {
  }	

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
	
	NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable CAN1 RX0 interrupt IRQ channel */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
