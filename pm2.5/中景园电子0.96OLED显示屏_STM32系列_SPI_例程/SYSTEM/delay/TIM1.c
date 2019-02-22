//TIM1相关的配置文件


#include "TIM.h"


u16 CCR2_Val = 500;
u16 CCR3_Val = 500;//占空比，周期为1000




/* 配置TIM1复用输出PWM时用到的I/O */
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




/*配置TIM1输出的PWM信号的模式，如周期、极性、占空比 */
void TIM1_Mode_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_BDTRInitTypeDef      TIM1_BDTRInitStruct;
    TIM_OCInitTypeDef        TIM_OCInitStructure;

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 1000; //计数周期，向上记到此数，计数值清零
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1;//定时器分频系数,Ftimer = 72M/(TIM_Prescaler+1) = 1ms
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//与死区时间分频有关
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 2;//向上计数模式
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
    /*配置BDTR寄存器，配置死区时间*/
    /*
       定时器时钟72M   TIM_ClockDivision = TIM_CKD_DIV1时,  Tdts = 13.89ns
       0 - 1.764us 
       1.778us - 3.505us 
       3.556us - 7.000us 
       7.1117us - 14us 
       需要更长时间，使用TIM_ClockDivision
    */
    TIM1_BDTRInitStruct.TIM_OSSRState = TIM_OSSRState_Disable;
    TIM1_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Disable;
    TIM1_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM1_BDTRInitStruct.TIM_DeadTime = 205; //死区时间  72:1us 172:3us 205:5us
    TIM_BDTRConfig(TIM1,&TIM1_BDTRInitStruct);
 
//    TIM1->BDTR |= 72;   //设置死区时间，上面方法可以，这种快且简单
    
     /* PWM1 Mode configuration: Channel2 */
#ifndef UART1		 
     TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;//PWM2模式
     TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
     TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//比较互补输出使能
     TIM_OCInitStructure.TIM_Pulse = CCR2_Val;   //比较值，占空比
     TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //输出极性
     TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//互补输出极性
     TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//空闲状态下的TIM输出比较的引脚状态
     TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;//空闲状态下的TIM互补输出的引脚状态
     TIM_OC2Init(TIM1, &TIM_OCInitStructure);   //初始化通道二比较输出
     TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);      //配置通道二，自动重装载使能
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

    TIM_ARRPreloadConfig(TIM1, ENABLE);//重载装栽值 ENABLE 立即生效,DISABLE 下一个比较周期生效

    /* TIM1 enable counter */
    TIM_Cmd(TIM1, ENABLE);//使能定时器1
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);//使能PWM外围输出   
}

void PWM_TIM1(uint16_t arr,uint16_t psc)
{
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_BDTRInitTypeDef      TIM1_BDTRInitStruct;
    TIM_OCInitTypeDef        TIM_OCInitStructure;
	  GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);    //???TIM1????
    TIM_DeInit(TIM1);

   //设置在下一个更新时间装入活动的自动重载寄存器
     TIM_TimeBaseStructure.TIM_Period =arr;     
    //设置TIMx时钟频率出书的预分频值
     TIM_TimeBaseStructure.TIM_Prescaler =psc;              
    //设置时钟分割
     TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    //设置计数模式
     TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
     //初始化参数
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

    TIM_Cmd(TIM1, ENABLE);  //使能TIM1
}
void TIM1_UP_IRQHandler()
{
	STEP_MOTION_IRQHandler(1);
}


