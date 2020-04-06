#include "main.h"
/*
NVIC_PriorityGroup_2
CAN_interrupt 1/1
LS_interrupt 1/2
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
		TIM_SetCompare1(TIM1,0);
		TIM_SetCompare2(TIM1,0);				
		TIM_CtrlPWMOutputs(TIM1,DISABLE);
		CanWriteData(0, mode_error);
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);	
	}
}

