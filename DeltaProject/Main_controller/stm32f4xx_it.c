#include "main.h"
/*
CAN_Interrupt 0/1
TIM_T_interrupt 0/0
Uart_dma_interrupt 0/2
Send_PC 0/3

	txbuff[0] = 'a';
	DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
	DMA1_Stream3->NDTR = 1;
	DMA_Cmd(DMA1_Stream3, ENABLE);
*/

#define Mode_Auto_stand 'i'
#define Mode_Auto_run 'a'
#define Mode_Manual 'm'
#define Home 'h'
#define Pick 'p'
#define Drop 'd'
#define Emergency 'e'
#define location 'l'
#define RunAGV 'r'
#define StopAGV 's'
#define Finish_pick_moderun 'f'
char mode_run = Mode_Manual;
uint8_t Manual_stt = 0;
double timef_manual = 1.0;
double pre_location_manual[3]={0.0, 0.0,  -456.6461}, location_manual[3] = {0.0, 0.0, 0.0} ; 
double pre_location_Auto_run[3]={0.0, 0.0,  -456.6461}, location_Auto_run[3] = {0.0, 0.0, 0.0} ; 
double timef_Auto_run = 1.0;


extern uint8_t set_home_complete[3];
extern uint8_t rxbuff[] ;
extern uint8_t txbuff[] ;
extern uint8_t txbuff_AGV[];
double Position_SP1 = 0, Position_SP2=0, Position_SP3=0;


#define type_cir (uint8_t)0
#define type_squ (uint8_t)1
#define type_tri (uint8_t)2
uint8_t type_object = 0, pre_type_object = 1;
enum step{step1=0,step2,step3,step4,step5,step6};
enum step stepx;

double p0dot[3] = {0,0,0}, pfdot[3] = {0,0,0};
#define time0 (double)0.0
#define timef1 (double)1.2
#define timef2 (double)0.7
#define tim_planning (double)0.01
double t = time0;
	
#define height1 (double)150.0  // 650 - 500  
#define height2	(double)100.0   // 600 - 500
double location_object[3] = {0.0,0.0,0.0};
double location_object_medium[3] = {0.0,0.0,0.0};
double location_box_medium[3] = {0.0, 0.0, 0.0};
double location_box_cir[3] = {100.0,250.0,-600.0};
double location_box_squ[3] = {0.0,250.0,-600.0};
double location_box_tri[3] = {-100.0,250.0,-600.0};

struct planning planning1, planning2, planning3, planning4, planning5, planning6;



void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		static double X=0, Y=0, Z=0;
		struct planning planning_step;
		if(stepx == step1)
		{planning_step = planning1;}
		else if(stepx == step2)
		{planning_step = planning2;}
		else if(stepx == step3)
		{planning_step = planning3;}
		else if(stepx == step4)
		{planning_step = planning4;}
		else if(stepx == step5)
		{planning_step = planning5;}
		else if(stepx == step6)
		{planning_step = planning6;}
		else TIM_Cmd(TIM7, DISABLE);

		X = planning_step.a0[0] + planning_step.a1[0]*t  + planning_step.a2[0]*pow(t,2) + planning_step.a3[0]*pow(t,3) +  planning_step.a4[0]*pow(t,4) + planning_step.a5[0]*pow(t,5);
		Y = planning_step.a0[1] + planning_step.a1[1]*t  + planning_step.a2[1]*pow(t,2) + planning_step.a3[1]*pow(t,3) +  planning_step.a4[1]*pow(t,4) + planning_step.a5[1]*pow(t,5);
		Z = planning_step.a0[2] + planning_step.a1[2]*t  + planning_step.a2[2]*pow(t,2) + planning_step.a3[2]*pow(t,3) +  planning_step.a4[2]*pow(t,4) + planning_step.a5[2]*pow(t,5);
		double *theta = Inverse_kinematic(X, Y, Z);
		Position_SP1 = *theta; 
		Position_SP2 = *(theta+1); 
		Position_SP3 = *(theta+2);
		CanWriteData(*(theta+2),mode_send_position3);
		CanWriteData(*(theta+1),mode_send_position2);
		CanWriteData(*theta ,mode_send_position1);
	
		switch(mode_run)
		{
			case Mode_Auto_stand:  //////// stand pick object
			{
				if(stepx==step1 || stepx==step4)
				{
					if(t<timef1)
					{
					t+=tim_planning;
					}
					else
					{
						t = 0;
						if(stepx == step1)
						{
							stepx = step2; 			
						}
						else if(stepx == step4)
						{
							stepx = step5;
						}
						else TIM_Cmd(TIM7, DISABLE);
					}
				}
				else
				{
					if(t<timef2)
					{
					t+=tim_planning;
					}
					else
					{
						t = 0;
						if(stepx == step2)
						{
							stepx = step3;
							GPIO_ResetBits(GPIOD, GPIO_Pin_12);
						}
						else if(stepx == step3)
						{
							stepx = step4;
						}
						else if(stepx == step5)
						{
							stepx = step6;
							GPIO_SetBits(GPIOD, GPIO_Pin_12);
						}
						else if(stepx == step6)
						{
							t = time0;
							TIM_Cmd(TIM2, ENABLE);
							TIM_Cmd(TIM7, DISABLE);
						}	
						else TIM_Cmd(TIM7, DISABLE);
					}			
				}
				break;
			}
			case Mode_Auto_run: /////// Communicate AGV
			{
					if(t<timef_Auto_run)
					{
						t+=tim_planning;
					}
					else
					{
						t = time0;
						//// Send to ras : infrom picked object
						txbuff[0] = Finish_pick_moderun;
						DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
						DMA1_Stream3->NDTR = 1;
						DMA_Cmd(DMA1_Stream3, ENABLE);
						
						/////////////
						TIM_Cmd(TIM7, DISABLE);
					}
					break;
			}
			case Mode_Manual: ////////Manual
			{
					if(t<timef_manual)
					{
						t+=tim_planning;
					}
					else
					{
						t = time0;
						TIM_Cmd(TIM7, DISABLE);
					}
					break;
			}
			default: break;
		}
 }
}



// Ngat nhan UART
void DMA1_Stream1_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1) != RESET)
	{
		
		if(rxbuff[0] == Mode_Auto_stand) // auto and stand
		{			
			if(mode_run != Mode_Auto_stand)
			{
				mode_run = Mode_Auto_stand;
				GPIO_SetBits(GPIOD, GPIO_Pin_12);
				TIM_Cmd(TIM7, DISABLE);
				//double *theta = Inverse_kinematic(location_box_squ[0], location_box_squ[1],location_box_squ[2]+height2);
				CanWriteData(40.0, mode_send_position3);
				CanWriteData(40.0, mode_send_position2);
				CanWriteData(40.0, mode_send_position1);	
				delay(300000);
				TIM_Cmd(TIM2, ENABLE);
			}
		}
		else if(rxbuff[0] == Mode_Auto_run) // auto and run
		{			
			if(mode_run != Mode_Auto_run)
			{
				mode_run = Mode_Auto_run;
				GPIO_SetBits(GPIOD, GPIO_Pin_12);
				TIM_Cmd(TIM7, DISABLE);
				CanWriteData(40.0, mode_send_position3);
				CanWriteData(40.0, mode_send_position2);
				CanWriteData(40.0, mode_send_position1);	
				pre_location_Auto_run[0] = 0.0;
				pre_location_Auto_run[1] = 0.0;
			  pre_location_Auto_run[2] =  -456.6461;
				delay(300000);
			}
		}
		if(rxbuff[0] == Mode_Manual) // manual
		{		
			if(mode_run != Mode_Manual)
			{
				mode_run = Mode_Manual;
				Manual_stt = 0;
				GPIO_SetBits(GPIOD, GPIO_Pin_12);
				TIM_Cmd(TIM7, DISABLE);
				CanWriteData(40.0,mode_send_position3);
				CanWriteData(40.0,mode_send_position2);
				CanWriteData(40.0,mode_send_position1);
				pre_location_manual[0] = 0.0;
				pre_location_manual[1] = 0.0;
			  pre_location_manual[2] =  -456.6461;
				delay(300000);
			}
		}
		else if(rxbuff[0] == Home) // finish
		{
			mode_run = Home;
			TIM_Cmd(TIM7, DISABLE);
			CanWriteData(40.0,mode_send_position3);
			CanWriteData(40.0,mode_send_position2);
			CanWriteData(40.0,mode_send_position1);
			pre_location_manual[0] = 0;
			pre_location_manual[1] = 0;
			pre_location_manual[2] =  -456.6461;
		}
		else if(rxbuff[0] == Pick) // pick 
		{
			if(mode_run == Mode_Manual)
			{
				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
			}
		}
		else if(rxbuff[0] == Drop) // drop
		{
			if(mode_run == Mode_Manual)
			{
				GPIO_SetBits(GPIOD, GPIO_Pin_12);
			}
		}
		else if(rxbuff[0] == Emergency) // emer
		{
			TIM_Cmd(TIM7, DISABLE);
			TIM_Cmd(TIM2, DISABLE);
			CanWriteData(0.0,mode_error);
			CanWriteData(0.0,mode_error);
			CanWriteData(0.0,mode_error);
			USART_Cmd(USART3, DISABLE);
		}
		else if(rxbuff[0] == RunAGV) // e
		{
			if(mode_run == Mode_Auto_run)
			{
				GPIO_SetBits(GPIOD, GPIO_Pin_12);
				CanWriteData(40.0,mode_send_position3);
				CanWriteData(40.0,mode_send_position2);
				CanWriteData(40.0,mode_send_position1);
				pre_location_Auto_run[0] = 0.0;
				pre_location_Auto_run[1] = 0.0;
			  pre_location_Auto_run[2] =  -456.6461;
				
				/* Send data to AGV run */
				txbuff_AGV[0] = 'r'; txbuff_AGV[1] = 'u'; txbuff_AGV[2] = 'n'; txbuff_AGV[3] = '^';
				DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
				DMA1_Stream4->NDTR = 4;
				DMA_Cmd(DMA1_Stream4, ENABLE);			
				//////////////////////////
			}
		}
		else if(rxbuff[0] == StopAGV) // e
		{
			if(mode_run == Mode_Auto_run)
			{
				GPIO_SetBits(GPIOD, GPIO_Pin_12);
				/* Send data to AGV stop */				
				txbuff_AGV[0] = 's'; txbuff_AGV[1] = 't'; txbuff_AGV[2] = 'o'; txbuff_AGV[3] = '^';
				DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
				DMA1_Stream4->NDTR = 4;
				DMA_Cmd(DMA1_Stream4, ENABLE);
				
				//////////// Send to ras : inform AGV stopped
				txbuff[0] = StopAGV;
				DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
				DMA1_Stream3->NDTR = 1;
				DMA_Cmd(DMA1_Stream3, ENABLE);
			
			}
		}
		else if(rxbuff[0] == location)
		{
			TIM_Cmd(TIM7, DISABLE);
			TIM_Cmd(TIM2, DISABLE);
			if(mode_run == Mode_Auto_stand)
			{		
				type_object = (rxbuff[1]-48);
				location_object[0] = (int16_t)((rxbuff[3]-48)*100 + (rxbuff[4]-48)*10 + (rxbuff[5]-48)); 
				location_object[1] = (int16_t)((rxbuff[7]-48)*100 + (rxbuff[8]-48)*10 + (rxbuff[9]-48)); 
				if(rxbuff[2]=='1') location_object[0] = -location_object[0];
				if(rxbuff[6]=='1') location_object[1] = -location_object[1];
				location_object[2] = -650.0;
				
				location_object_medium[0] = location_object[0];
				location_object_medium[1] = location_object[1];
				location_object_medium[2] = location_object[2] + height1; 
				
				if(Manual_stt == 0)
				{
					location_box_medium[0] = 0;
					location_box_medium[1] = 0;
					location_box_medium[2] = -456.6461;
					Manual_stt = 1;
				}
				else
				{
					switch (pre_type_object)
					{
						case type_cir:
							{ 
							location_box_medium[0] = location_box_cir[0];
							location_box_medium[1] = location_box_cir[1];
							location_box_medium[2] = location_box_cir[2] + height2;
							break;
							}
						case type_squ: 
							{
							location_box_medium[0] = location_box_squ[0];
							location_box_medium[1] = location_box_squ[1];
							location_box_medium[2] = location_box_squ[2] + height2;
							break;
							}
						default : 
							{
							location_box_medium[0] = location_box_tri[0];
							location_box_medium[1] = location_box_tri[1];
							location_box_medium[2] = location_box_tri[2] + height2;				
							break;
							}
					}
				}
				planning1 = paramsPoly_inverse(location_box_medium, location_object_medium, p0dot, pfdot, time0, timef1);		
				planning2 = paramsPoly_inverse(location_object_medium, location_object, p0dot, pfdot, time0, timef2);
				planning3 = paramsPoly_inverse(location_object, location_object_medium, p0dot, pfdot, time0, timef2);

				switch (type_object)
				{
					case type_cir:
						{ 
						location_box_medium[0] = location_box_cir[0];
						location_box_medium[1] = location_box_cir[1];
						location_box_medium[2] = location_box_cir[2] + height2;
						planning4 = paramsPoly_inverse(location_object_medium, location_box_medium , p0dot, pfdot, time0, timef1);
						planning5 = paramsPoly_inverse(location_box_medium, location_box_cir , p0dot, pfdot, time0, timef2);
						planning6 = paramsPoly_inverse(location_box_cir, location_box_medium  , p0dot, pfdot, time0, timef2);
						break;
						}
					case type_squ: 
						{
						location_box_medium[0] = location_box_squ[0];
						location_box_medium[1] = location_box_squ[1];
						location_box_medium[2] = location_box_squ[2] + height2;
						planning4 = paramsPoly_inverse(location_object_medium, location_box_medium , p0dot, pfdot, time0, timef1);
						planning5 = paramsPoly_inverse(location_box_medium, location_box_squ , p0dot, pfdot, time0, timef2);
						planning6 = paramsPoly_inverse(location_box_squ, location_box_medium  , p0dot, pfdot, time0, timef2);
						break;
						}
					default: 
						{
						location_box_medium[0] = location_box_tri[0];
						location_box_medium[1] = location_box_tri[1];
						location_box_medium[2] = location_box_tri[2] + height2;	
						planning4 = paramsPoly_inverse(location_object_medium, location_box_medium , p0dot, pfdot, time0, timef1);
						planning5 = paramsPoly_inverse(location_box_medium, location_box_tri , p0dot, pfdot, time0, timef2);
						planning6 = paramsPoly_inverse(location_box_tri, location_box_medium  , p0dot, pfdot, time0, timef2);					
						break;
						}
				}	
				pre_type_object = type_object;
				t = time0;
				stepx = step1;
	      TIM_Cmd(TIM7, ENABLE);
			}
			else if(mode_run == Mode_Manual)
			{
				TIM_Cmd(TIM7, DISABLE);
				location_manual[0] = (int16_t)((rxbuff[3]-48)*100 + (rxbuff[4]-48)*10 + (rxbuff[5]-48)); 
				location_manual[1] = (int16_t)((rxbuff[7]-48)*100 + (rxbuff[8]-48)*10 + (rxbuff[9]-48)); 
				location_manual[2] = (int16_t)((rxbuff[11]-48)*100 + (rxbuff[12]-48)*10 + (rxbuff[13]-48)); 
				if(rxbuff[2]=='1') location_manual[0] = -location_manual[0];
				if(rxbuff[6]=='1') location_manual[1] = -location_manual[1];			
				if(rxbuff[10]=='1') location_manual[2] = -location_manual[2];	
				timef_manual = (double)((int8_t)rxbuff[14])/10;
				planning1 = paramsPoly_inverse(pre_location_manual, location_manual, p0dot, pfdot, time0, timef_manual);	
				pre_location_manual[0] = location_manual[0];
				pre_location_manual[1] = location_manual[1];
				pre_location_manual[2] = location_manual[2];			
				t = time0;
				stepx = step1;				
				TIM_Cmd(TIM7, ENABLE);
			}
			else if(mode_run == Mode_Auto_run)
			{
				TIM_Cmd(TIM7, DISABLE);
				location_Auto_run[0] = (int16_t)((rxbuff[3]-48)*100 + (rxbuff[4]-48)*10 + (rxbuff[5]-48)); 
				location_Auto_run[1] = (int16_t)((rxbuff[7]-48)*100 + (rxbuff[8]-48)*10 + (rxbuff[9]-48)); 
				location_Auto_run[2] = (int16_t)((rxbuff[11]-48)*100 + (rxbuff[12]-48)*10 + (rxbuff[13]-48)); 
				if(rxbuff[2]=='1') location_Auto_run[0] = -location_Auto_run[0];
				if(rxbuff[6]=='1') location_Auto_run[1] = -location_Auto_run[1];			
				if(rxbuff[10]=='1') location_Auto_run[2] = -location_Auto_run[2];	
				timef_Auto_run = (double)((int8_t)rxbuff[14])/10;
				planning1 = paramsPoly_inverse(pre_location_Auto_run, location_Auto_run, p0dot, pfdot, time0, timef_Auto_run);	
				pre_location_Auto_run[0] = location_Auto_run[0];
				pre_location_Auto_run[1] = location_Auto_run[1];
				pre_location_Auto_run[2] = location_Auto_run[2];			
				t = time0;
				stepx = step1;				
				TIM_Cmd(TIM7, ENABLE);
			}
		}	
  DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
  DMA_Cmd(DMA1_Stream1, ENABLE);
	}
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		//////////////////////send
	if(mode_run == Mode_Auto_stand)
	{
	txbuff[0] = Mode_Auto_stand;
	DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
	DMA1_Stream3->NDTR = 1;
	DMA_Cmd(DMA1_Stream3, ENABLE);
	//////////////////////
	}
	}
}

void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line1);	
		CanWriteData(0, mode_complete_home1);
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
		NVIC_Init(&NVIC_InitStructure);
		set_home_complete[0] = 1;
	}
}

void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line2);	
		CanWriteData(0, mode_complete_home2);
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
		NVIC_Init(&NVIC_InitStructure);
		set_home_complete[1] = 1;
	}
}

void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line3);	
		CanWriteData(0, mode_complete_home3);
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
		NVIC_Init(&NVIC_InitStructure);
		set_home_complete[2] = 1;
	}
}