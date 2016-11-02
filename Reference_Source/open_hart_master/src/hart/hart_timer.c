#include "hart_driver.h"

unsigned char timer_init(unsigned int time_100us)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
//TIM_OCInitTypeDef  TIM_OCInitStructure;

   /* TIM6 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = time_100us;    	
  TIM_TimeBaseStructure.TIM_Prescaler = (SYSCLK_FREQUENCY/TICKS_SECOND)-1;	//Ԥ��Ƶֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//ʱ�ӷָ�
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;	//TIM���¼���ģʽ
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM2,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	
  TIM_ClearFlag(TIM2, TIM_FLAG_Update); //�������жϱ�־
  TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
  TIM_Cmd(TIM2,DISABLE);

  return TRUE;
}

void timer_enable(unsigned int time_limit)
{
	TIM_ClearFlag(TIM2, TIM_FLAG_Update); //�������жϱ�־
  TIM_SetCounter(TIM2,time_limit);		
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM2,ENABLE);
}

void timer_disable(void)
{
	TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
  TIM_Cmd(TIM2,DISABLE);
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)
	{
		TIM_ClearFlag(TIM2, TIM_FLAG_Update); //�������жϱ�־
		//funtion
	}
}


