#include "main.h"
/*
NVIC_PriorityGroup_2
CAN_interrupt 1/1
LS_interrupt 2/2
encoder_interrupt 0/1
T_interrupt 1/0

*/
extern uint32_t CAN_ID;
extern int32_t Rotary, Counter;
extern double Set_Position, Position;
extern uint32_t CAN_ID;



void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		if(TIM_GetCounter(TIM3) >= 30000)
		{
		Rotary--;
		}
		else
		{
		Rotary++;
		}
	}	
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    STR_Position();
	}
}

void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line0);	
		TIM_Cmd(TIM2,DISABLE);
		TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
		TIM_SetCompare1(TIM1,0);
		TIM_SetCompare2(TIM1,0);				
		TIM_CtrlPWMOutputs(TIM1,DISABLE);
		CanWriteData(0, mode_error);
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);	
	}
	//delay(100000);
	//if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == RESET)
//		if(CAN_ID == 0x100)
//		{
//			static uint16_t f_o_i=0;
//			if(f_o_i%2 == 0 )
//			{
//			TIM_SetCompare1(TIM1,0);
//			TIM_SetCompare2(TIM1,500);			
//			}
//			else
//			{
//			TIM_SetCompare1(TIM1,500);
//			TIM_SetCompare2(TIM1,0);				
//			}
//			f_o_i++;
//		}
//		else

	}

