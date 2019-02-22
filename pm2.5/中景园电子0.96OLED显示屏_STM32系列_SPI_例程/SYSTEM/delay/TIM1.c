//TIM1��ص������ļ�


#include "TIM.h"


u16 CCR2_Val = 500;
u16 CCR3_Val = 500;//ռ�ձȣ�����Ϊ1000




/* ����TIM1�������PWMʱ�õ���I/O */
void TIM1_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* GPIOA and GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

#ifndef UART1	
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif
	
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_12);
}




/*����TIM1�����PWM�źŵ�ģʽ�������ڡ����ԡ�ռ�ձ� */
void TIM1_Mode_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_BDTRInitTypeDef      TIM1_BDTRInitStruct;
    TIM_OCInitTypeDef        TIM_OCInitStructure;

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 1000; //�������ڣ����ϼǵ�����������ֵ����
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1;//��ʱ����Ƶϵ��,Ftimer = 72M/(TIM_Prescaler+1) = 1ms
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//������ʱ���Ƶ�й�
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���ģʽ
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 2;//���ϼ���ģʽ
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
    /*����BDTR�Ĵ�������������ʱ��*/
    /*
       ��ʱ��ʱ��72M   TIM_ClockDivision = TIM_CKD_DIV1ʱ,  Tdts = 13.89ns
       0 - 1.764us 
       1.778us - 3.505us 
       3.556us - 7.000us 
       7.1117us - 14us 
       ��Ҫ����ʱ�䣬ʹ��TIM_ClockDivision
    */
    TIM1_BDTRInitStruct.TIM_OSSRState = TIM_OSSRState_Disable;
    TIM1_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Disable;
    TIM1_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM1_BDTRInitStruct.TIM_DeadTime = 205; //����ʱ��  72:1us 172:3us 205:5us
    TIM_BDTRConfig(TIM1,&TIM1_BDTRInitStruct);
 
//    TIM1->BDTR |= 72;   //��������ʱ�䣬���淽�����ԣ����ֿ��Ҽ�
    
     /* PWM1 Mode configuration: Channel2 */
#ifndef UART1		 
     TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;//PWM2ģʽ
     TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
     TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//�Ƚϻ������ʹ��
     TIM_OCInitStructure.TIM_Pulse = CCR2_Val;   //�Ƚ�ֵ��ռ�ձ�
     TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //�������
     TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//�����������
     TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//����״̬�µ�TIM����Ƚϵ�����״̬
     TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;//����״̬�µ�TIM�������������״̬
     TIM_OC2Init(TIM1, &TIM_OCInitStructure);   //��ʼ��ͨ�����Ƚ����
     TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);      //����ͨ�������Զ���װ��ʹ��
#endif

    /* PWM1 Mode configuration: Channel3 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;      
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);    

    TIM_ARRPreloadConfig(TIM1, ENABLE);//����װ��ֵ ENABLE ������Ч,DISABLE ��һ���Ƚ�������Ч

    /* TIM1 enable counter */
    TIM_Cmd(TIM1, ENABLE);//ʹ�ܶ�ʱ��1
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);//ʹ��PWM��Χ���   
}

void PWM_TIM1(uint16_t arr,uint16_t psc)
{
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_BDTRInitTypeDef      TIM1_BDTRInitStruct;
    TIM_OCInitTypeDef        TIM_OCInitStructure;
	  GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);    //???TIM1????
    TIM_DeInit(TIM1);

   //��������һ������ʱ��װ�����Զ����ؼĴ���
     TIM_TimeBaseStructure.TIM_Period =arr;     
    //����TIMxʱ��Ƶ�ʳ����Ԥ��Ƶֵ
     TIM_TimeBaseStructure.TIM_Prescaler =psc;              
    //����ʱ�ӷָ�
     TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    //���ü���ģʽ
     TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
     //��ʼ������
     TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);               
/*
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);   //TIM1channel2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_2);  //TIM1channel3
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
//PA9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
    GPIO_Init(GPIOA, &GPIO_InitStructure); 

//PA10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
*/

    //TIM1?channel2  
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);  

      //TIM1?channel3
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);

    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);      

    TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1
}
void TIM1_UP_IRQHandler()
{
	STEP_MOTION_IRQHandler(1);
}


