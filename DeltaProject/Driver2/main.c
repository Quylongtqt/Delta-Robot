#include "main.h"

unsigned int i,j,k; // Counter variable for if function
double Temp_mul = 0;
extern double theta[6][6],P[6][6],theta_temp[6][6] ;
extern int32_t Counter,Rotary;
extern double Position,Set_Position;
void delay(uint32_t t)
{
	while(t--){};
}

int main(void)
{
	  Config_RCC();
	Initial_Condition_Position();
	Can_config();
	Config_Pro_LS_IO();
	TIM1_Config_PWM();
	TIM3_Config_Encoder();
	TIM2_Config_Sample_time();
//	SysTick_Config(SystemCoreClock/20);// Enable SysTick at 50ms interrupt
	
	while(1)
	{
	}
	return 0;
}

void Initial_Condition_Position(void) 
{	
	theta[0][0]= -1;
	theta[1][0]= 0;
	theta[2][0]= 0;
	theta[3][0]= 1;
	theta[4][0]= 0;
	theta[5][0]= 0;
	// Init P matrix
	P[0][0] = 800;
	P[1][1] = 800;
	P[2][2] = 800;
	P[3][3] = 800;
	P[4][4] = 800;
	P[5][5] = 800;	
}

void Config_RCC(void)
{
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	// Wait until HSE ready to use or not
	ErrorStatus errorStatus = RCC_WaitForHSEStartUp();
	if (errorStatus == SUCCESS)
	{
	//Enable prefetch Buffer
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		// Flash 2 wait state
		FLASH_SetLatency(FLASH_Latency_2);
		//LLCLK = clock in
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE);
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
		// HCLKK = SYSCLK
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		//PCLK2 = HCLK
		RCC_PCLK2Config(RCC_HCLK_Div1);
		//PCLK1 = HCLK/2
		RCC_PCLK1Config(RCC_HCLK_Div2);
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08);
	}
	else while(1);
}
		

void TIM2_Config_Sample_time(void)//////////////0.005s
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;
	TIM_TimeBaseStructure.TIM_Period = 50 - 1;		
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2,DISABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the TIM2 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

 /* PWM
 - TIM1_CH1  pin (PA.08)
 - TIM1_CH2  pin (PA.09)
 */
void TIM1_Config_PWM()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 1800 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 2 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	// channel 1
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	//channel 2
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


void TIM3_Config_Encoder() 
{	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel	= TIM_Channel_1 | TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 15;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	//TIM_ICInitStructure.TIM_ICPrescaler = 
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	/* Configure the timer */
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);

	TIM_SetCounter(TIM3,0);
	/* TIM3 counter enable */
	TIM_Cmd(TIM3, ENABLE);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	
	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the TIM2 gloabal Interrupt */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	
	Rotary=0;
}

void Display_Matrix(double x[6][6], unsigned char m, unsigned char n)
{
  for (i=0;i<m;i++)
	{
		for(j=0;j<n;j++)
		printf(" %10.2f",x[i][j]);
		printf("\n");
	}
	printf("\n");
}

void Multiply_Matrix(double x[6][6], unsigned char m, unsigned char n,  double y[6][6], unsigned char p, unsigned char q,  double z[6][6])
{
    for ( i = 0 ; i < m ; i++ )
    {
      for ( j = 0 ; j < q ; j++ )
      {
        for ( k = 0 ; k < p ; k++ )
        {
          Temp_mul = Temp_mul + x[i][k]*y[k][j];
        }
        z[i][j] = Temp_mul;
        Temp_mul = 0;
      }
    }
}
void Add_Matrix(double x[6][6], double y[6][6],unsigned char m, unsigned char n,  double z[6][6])
{
    for(i=0;i<m;i++)
       for(j=0;j<n;j++)
            z[i][j]=x[i][j]+y[i][j];
}
void Sub_Matrix(double x[6][6], double y[6][6],unsigned char m, unsigned char n,  double z[6][6])
{
    for(i=0;i<m;i++)
       for(j=0;j<n;j++)
            z[i][j]=x[i][j]-y[i][j];
}
void Multiply_Matrix_Vector(double x[6][6],unsigned char m, unsigned char n,double y, double z[6][6])
{
		   for(i=0;i<m;i++)
       for(j=0;j<n;j++)
            z[i][j]=x[i][j]*y;
}
void Divide_Matrix_Vector(double x[6][6],unsigned char m, unsigned char n,double y, double z[6][6])
{
	    for(i=0;i<m;i++)
       for(j=0;j<n;j++)
            z[i][j]=x[i][j]/y;
}
void Tranpose_Matrix(double x[6][6],unsigned char m, unsigned char n,  double y[6][6])
{
   for(i=0;i<n;i++)
      for(j=0;j<m;j++)
           y[i][j]=0;
  for(i=0;i<n;i++)
	{
    for(j=0;j<m;j++)
		{
         y[i][j]=x[j][i];
    }
	}
}