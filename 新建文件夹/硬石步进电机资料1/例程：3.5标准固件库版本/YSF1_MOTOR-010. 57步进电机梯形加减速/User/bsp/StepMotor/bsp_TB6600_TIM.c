/**
  ******************************************************************************
  * 文件名程: bsp_TB6600_TIM.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.1
  * 编写日期: 2017-04-27
  * 功    能: 57步进电机驱动实现
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F1Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp/StepMotor/bsp_TB6600_TIM.h"
#include "bsp/usart/bsp_debug_usart.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
uint32_t accel      = 5;       // 加速度
uint32_t decel      = 5;       // 减速度
int32_t  stpdecel   = 200;       // 急停减速度
uint8_t  LmtSnsPos  = 0;           // 正限位有效电平
uint8_t  LmtSnsNeg  = 0;           // 负限位有效电平
uint8_t  HomeSns    = 0;           // 回零有效电平
int32_t  position   = 0;           // 当前位置
int32_t  HomePos    = 0;           // 零点位置
int32_t  SoftLmtPos = 0x7fffffff;
int32_t  SoftLmtNeg = 0xffffffff;

bool bLmtPos        = FALSE; // 正限位当前状态
bool bLmtNeg        = FALSE; // 负限位当前状态
bool bStopCmd       = FALSE; // 停止命令
bool bEmgStopping   = FALSE; // 是否碰限位急停
bool bEnableSoftLmt = FALSE; // 是否允许软件限位
bool bHomeOK        = FALSE; // 当前是否处于零点状态
bool bZeroCapture   = FALSE; // 零点寻找

int32_t  ZeroDir           = POSITIVE;	  //回零方向
int32_t  ZeroOffset        = 0;           //零点偏移
uint32_t ZeroBackDistance  = 6400;        //回零回退距离 
uint8_t  ZeroStep          = FASTSEEKBACK;	
uint32_t HomeFastSpeed     = 200;        //回零快搜速度
uint32_t HomeSlowSpeed     = 100;         //回零慢搜速度

speedRampData srd;

uint8_t MotionStatus=0;//是否在运动？0：停止，1：运动


/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
//开方
uint32_t axis_sqrt(uint32_t x)
{
  register uint32_t xr;  // result register
  register uint32_t q2;  // scan-bit register
  register uint8_t f;   // flag (one bit)

  xr = 0;                     // clear result
  q2 = 0x40000000L;           // higest possible result bit
  do
  {
    if((xr + q2) <= x)
    {
      x -= xr + q2;
      f = 1;                  // set flag
    }
    else{
      f = 0;                  // clear flag
    }
    xr >>= 1;
    if(f){
      xr += q2;               // test flag
    }
  } while(q2 >>= 2);          // shift twice
  if(xr < x){
    return xr +1;             // add for rounding
  }
  else{
    return xr;
	  }
}

void EnableHomeCapture(void) // 使能回零捕获中断
{
  EXTI->IMR |=AXIS_HOME_1;
}

void DisableHomeCapture(void) // 禁用回零捕获中断
{
  EXTI->IMR &=~AXIS_HOME_1;
}

/**
  * 函数功能: 配置TIMx复用输出PWM时用到的I/O
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
static void StepMotor_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
	RCC_APB2PeriphClockCmd(AXIS_PULSE_CLK|AXIS_DIR_CLK|AXIS_LMTPOS_CLK|
                         AXIS_LMTNEG_CLK|AXIS_HOME_CLK|RCC_APB2Periph_AFIO, ENABLE);

  GPIO_InitStructure.GPIO_Pin = AXIS_PULSE_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(AxisPulsePort, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = AXIS_DIR_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(AxisDirPort, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin=AXIS_LMTPOS_1;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
  GPIO_Init(AxisLmtPosPort, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin=AXIS_LMTNEG_1;
  GPIO_Init(AxisLmtNegPort, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin=AXIS_HOME_1;
  GPIO_Init(AxisHomePort, &GPIO_InitStructure);
}


static void StepMotor_NVIC_Configuration(void)//中断向量配置
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  /* Enable the AXIS_TIMx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = AXIS_TIMx_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = AxisEXTI_IRQCHANNEL;		//使能按键所在的外部中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  	//先占优先级2位,共4级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					//先占优先级2位,从优先级4位
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	NVIC_Init(&NVIC_InitStructure);	
}

static void StepMotor_EXTILine_Config(void)//外部中断配置
{
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_EXTILineConfig(AxisEXTI_PORTSource, AxisEXTIPinSource);

	EXTI_InitStructure.EXTI_Line = AxisEXTILine;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	
}

void StepMotor_TIMx_Init(void)//定时器配置
{ 	
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  AXIS_TIMX_CLOCKCMD(AXIS_TIMx_RCC_CLK, ENABLE);
  
  StepMotor_GPIO_Configuration();
  StepMotor_NVIC_Configuration();
  StepMotor_EXTILine_Config();  
  
  TIM_TimeBaseStructure.TIM_Period =0xFFFF;
  TIM_TimeBaseStructure.TIM_Prescaler =35;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(AXIS_TIMx, &TIM_TimeBaseStructure);

  /* Output Compare Active Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Active;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 100;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  AXIS_TIMx_OCnInit(AXIS_TIMx, &TIM_OCInitStructure);
	
	TIM_ClearFlag(AXIS_TIMx, TIM_FLAG_Update);

  /* TIM IT enable */
//  TIM_ITConfig(AXIS_TIMx, TIM_IT_Update, ENABLE);
  TIM_ITConfig(AXIS_TIMx, TIM_IT_CC1, ENABLE);
}

//限位开关检测，如果触发了限位开关(TRUE状态)，电机停止转动(停止输出脉冲信号)
void LimitDetect(void)
{
  if(GPIO_ReadInputDataBit(AxisLmtPosPort, AXIS_LMTPOS_1)) // 读取到正限位引脚为高电平
  {
    if(LmtSnsPos==0) // 如果定义低电平有效
    {
      bLmtPos=FALSE; // 未到达正限位
    }
    else             // 如果定义高电平有效
    {
      bLmtPos=TRUE;  // 触发了正限位
    }		
  }
  else                                                     // 读取到正限位引脚为低电平
  {
    if(LmtSnsPos==0) // 如果定义低电平有效
    {
      bLmtPos=TRUE;  // 触发了正限位
    }
    else             // 如果定义高电平有效
    {
      bLmtPos=FALSE; // 未到达正限位
    }
  }
  if(GPIO_ReadInputDataBit(AxisLmtNegPort, AXIS_LMTNEG_1)) // 读取到负限位引脚为高电平
  {
    if(LmtSnsNeg==0) // 如果定义低电平有效
    {
      bLmtNeg=FALSE; // 未到达负限位
    }
    else             // 如果定义高电平有效
    {
      bLmtNeg=TRUE;  // 触发了负限位
    }		
  }
  else                                                     // 读取到正限位引脚为低电平
  {
    if(LmtSnsNeg==0) // 如果定义低电平有效
    {
      bLmtNeg=TRUE;  // 触发了负限位
    }
    else             // 如果定义高电平有效
    {
      bLmtNeg=FALSE; // 未到达负限位
    }
  }
}

//轴回零程序，axis参数是轴号
void AxisHome(void)
{
	switch(ZeroStep)
	{
		case IDEL:
			break;
		case FASTSEEK:
			if(!MotionStatus)   //是否在运动？0：停止，1：运动
			{
        if(bZeroCapture)
        {
          ZeroStep=FASTSEEKBACK;
          bZeroCapture=FALSE;
          AxisMoveRel((position-HomePos+ZeroBackDistance)*ZeroDir*-1, accel, decel, HomeFastSpeed);
        }
        else
        {
          ZeroStep=IDEL;
        }
			}
			break;
		case FASTSEEKSTOP:
			break;
		case FASTSEEKBACK:
			if(!MotionStatus)
			{
				EnableHomeCapture();						
				AxisMoveRel(0x7FFFFFFF*ZeroDir, accel, decel, HomeSlowSpeed);	
				ZeroStep=SLOWSEEK;
			}
			break;
		case SLOWSEEK:
			if(!MotionStatus)
			{
				if(bZeroCapture)
				{
					ZeroStep=MOVETOZERO;
					bZeroCapture=FALSE;
					AxisMoveRel((position-HomePos+ZeroOffset)*ZeroDir*-1, accel, decel, HomeFastSpeed);
				}
				else
				{
					ZeroStep=IDEL;
				}
			}
			break;
		case SLOWSEEKSTOP:
			break;
		case MOVETOZERO:
			if(!MotionStatus)
			{
				ZeroStep=IDEL;
				position=0;
			}
			break;
	}
}

/*! \brief Move the stepper motor a given number of steps.
 *
 *  Makes the stepper motor move the given number of steps.
 *  It accelrate with given accelration up to maximum speed and decelerate
 *  with given deceleration so it stops at the given step.
 *  If accel/decel is to small and steps to move is to few, speed might not
 *  reach the max speed limit before deceleration starts.
 *
 *  \param step  Number of steps to move (pos - CW, neg - CCW).
 *  \param accel  Accelration to use, in 0.01*rad/sec^2.
 *  \param decel  Decelration to use, in 0.01*rad/sec^2.
 *  \param speed  Max speed, in 0.01*rad/sec.
 */
 /*signed int range:-2147483648---21474 83647
 **unsiged int range:0---4294967295u
 */
void AxisMoveAbs(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)//绝对运动，step是目标位置，accel是加速度，decel是减速度，speed是速度
{
  //! Number of steps before we hit max speed.达到最大速度时的步数
  uint32_t max_s_lim;
  //! Number of steps before we must start deceleration (if accel does not hit max speed).
  //如果加速没有达到最大速度，但是必须要开始减速的步数
  uint32_t accel_lim;
	float ftemp=0.0;

	step=step-position;
  if(step <0)
  {
    if(GPIO_ReadInputDataBit(AxisLmtNegPort, AXIS_LMTNEG_1))
    {
      if(LmtSnsNeg==0)
      {
        bLmtNeg=FALSE;
      }
      else
      {
        bLmtNeg=TRUE;
        return;
      }
          
    }
    else
    {
      if(LmtSnsNeg==0)
      {
        bLmtNeg=TRUE;
        return;
      }
      else
      {
        bLmtNeg=FALSE;	
      }		 	
    }
    srd.dir = CCW;
    GPIO_ResetBits(AxisDirPort, AXIS_DIR_1);
    step =-step;
  }
  else
  {
    if(GPIO_ReadInputDataBit(AxisLmtPosPort, AXIS_LMTPOS_1))
    {
      if(LmtSnsPos==0)
      {
        bLmtPos=FALSE;
      }
      else
      {
        bLmtPos=TRUE;
        return;
      }					
    }
    else
    {
      if(LmtSnsPos==0)
      {
        bLmtPos=TRUE;
        return;
      }
      else
      {
        bLmtPos=FALSE;
      }		 	
    }
    srd.dir = CW;
    GPIO_SetBits(AxisDirPort, AXIS_DIR_1);
  }

  if(step == 1)
  {
    srd.accel_count = -1; // Move one step...
    srd.run_state = DECEL;// ...in DECEL state.
    srd.step_delay = 1000;	// Just a short delay so main() can act on 'running'.	   
    AXIS_TIM_SetCompareN(AXIS_TIMx,100);
    TIM_SetAutoreload(AXIS_TIMx,100);
		MotionStatus = 1;
		TIM_Cmd(AXIS_TIMx, ENABLE);			 
  }
  else if(step != 0)  // Only move if number of steps to move is not zero.
  {
    // Refer to documentation for detailed information about these calculations.
    // Set max speed limit, by calc min_delay to use in timer.
    // min_delay = (alpha / tt)/ w
    srd.min_delay = T1_FREQ/speed/2;

    // Set accelration by calc the first (c0) step delay .
    // step_delay = 1/tt * axis_sqrt(2*alpha/accel)
    // step_delay = ( tfreq*0.676/100 )*100 * axis_sqrt( (2*alpha*10000000000) / (accel*100) )/10000
    srd.step_delay = ((long)T1_FREQ*0.676* axis_sqrt(2000000 / accel))/1000/2;
    // Find out after how many steps does the speed hit the max speed limit.
    // max_s_lim = speed^2 / (2*alpha*accel)
    max_s_lim = speed*speed/(2*accel);
    // If we hit max speed limit before 0,5 step it will round to 0.
    // But in practice we need to move atleast 1 step to get any speed at all.
    if(max_s_lim == 0){
      max_s_lim = 1;
    }

    // Find out after how many steps we must start deceleration.
    // n1 = (n1+n2)decel / (accel + decel)
    if((accel+decel)>step)
		{
//			accel_lim = step*decel/(accel+decel);
			ftemp=(float)decel/(float)(accel+decel);
			accel_lim = (float)step*ftemp;
		}
		else
		{
			accel_lim = step/(accel+decel)*decel;
		}
    // We must accelrate at least 1 step before we can start deceleration.
    if(accel_lim == 0){
      accel_lim = 1;
    }

    // Use the limit we hit first to calc decel.
    if(accel_lim <= max_s_lim){
      srd.decel_val = accel_lim - step;
    }
    else{
      srd.decel_val =-(int32_t)(max_s_lim*accel/decel);
    }
    // We must decelrate at least 1 step to stop.
    if(srd.decel_val == 0){
      srd.decel_val = -1;
    }

    // Find step to start decleration.
    srd.decel_start = step + srd.decel_val;

    // If the maximum speed is so low that we dont need to go via accelration state.
    if(srd.step_delay <= srd.min_delay)
	 {
      srd.step_delay = srd.min_delay;
      srd.run_state = RUN;
    }
    else{
      srd.run_state = ACCEL;
    }

    // Reset counter.
    srd.accel_count = 0;
    MotionStatus = 1;
    AXIS_TIM_SetCompareN(AXIS_TIMx,100);
    TIM_SetAutoreload(AXIS_TIMx,100);	
    // Set Timer/Counter to divide clock by 8
	  TIM_Cmd(AXIS_TIMx, ENABLE);
  }
}

//相对运动，step是目标位置，accel是加速度，decel是减速度，speed是速度
void AxisMoveRel(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
  //! Number of steps before we hit max speed.
  uint32_t max_s_lim;
  //! Number of steps before we must start deceleration (if accel does not hit max speed).
  uint32_t accel_lim;
	float ftemp=0.0;

  if(step <0) // 负方向运动
  {
    if(GPIO_ReadInputDataBit(AxisLmtNegPort, AXIS_LMTNEG_1)) // 负限位状态读取
    {
      if(LmtSnsNeg==0)
      {
        bLmtNeg=FALSE;
      }
      else
      {
        bLmtNeg=TRUE;
        return;
      }						
    }
    else
    {
      if(LmtSnsNeg==0)
      {
        bLmtNeg=TRUE;
        return;
      }
      else
      {
        bLmtNeg=FALSE;	
      }		 	
    }
    srd.dir = CCW;     // 负方向
    Axis_SETDIR_CCW(); // 控制电机方向
    step =-step;       // 得到步数绝对值
  }
  else // 正方向运动
  {
		if(GPIO_ReadInputDataBit(AxisLmtPosPort, AXIS_LMTPOS_1)) // 正限位状态读取
		{
			if(LmtSnsPos==0)
			{
				bLmtPos=FALSE;
			}
			else
			{
				bLmtPos=TRUE;
				return;
			}					
		}
		else
		{
			if(LmtSnsPos==0)
			{
				bLmtPos=TRUE;
				return;
			}
			else
			{
				bLmtPos=FALSE;
			}		 	
		}
    srd.dir = CW;     // 正方向
	  Axis_SETDIR_CW(); // 控制电机方向
  }

  if(step == 1) // 步数为 1
  {
    // Move one step...
    srd.accel_count = -1;
    // ...in DECEL state.
    srd.run_state = DECEL;
    // Just a short delay so main() can act on 'running'.
    srd.step_delay = 1000;
    AXIS_TIM_SetCompareN(AXIS_TIMx,100);
    TIM_SetAutoreload(AXIS_TIMx,100);
    
		MotionStatus = 1;		//是否在运动？0：停止，1：运动
		TIM_Cmd(AXIS_TIMx, ENABLE);	// 使能脉冲输出
  }
  // Only move if number of steps to move is not zero.
  else if(step != 0)
  {
    // Refer to documentation for detailed information about these calculations.

    // Set max speed limit, by calc min_delay to use in timer.
    // min_delay = (alpha / tt)/ w
    srd.min_delay = T1_FREQ/speed/2;

    // Set accelration by calc the first (c0) step delay .
    // step_delay = 1/tt * axis_sqrt(2*alpha/accel)
    // step_delay = ( tfreq*0.676/100 )*100 * axis_sqrt( (2*alpha*10000000000) / (accel*100) )/10000
    srd.step_delay = ((long)T1_FREQ*0.676* axis_sqrt(2000000 / accel))/1000/2;
    // Find out after how many steps does the speed hit the max speed limit.
    // max_s_lim = speed^2 / (2*alpha*accel)
    max_s_lim = speed*speed/(2*accel);
    // If we hit max speed limit before 0,5 step it will round to 0.
    // But in practice we need to move atleast 1 step to get any speed at all.
    if(max_s_lim == 0){
      max_s_lim = 1;
    }

    // Find out after how many steps we must start deceleration.
    // n1 = (n1+n2)decel / (accel + decel)
    if((accel+decel)>step)
		{
//			accel_lim = step*decel/(accel+decel);
			ftemp=(float)decel/(float)(accel+decel);
			accel_lim = (float)step*ftemp;
		}
		else
		{
			accel_lim = step/(accel+decel)*decel;
		}
    // We must accelrate at least 1 step before we can start deceleration.
    if(accel_lim == 0){
      accel_lim = 1;
    }

    // Use the limit we hit first to calc decel.
    if(accel_lim <= max_s_lim){
      srd.decel_val = accel_lim - step;
    }
    else{
      srd.decel_val =-(int32_t)(max_s_lim*accel/decel);
    }
    // We must decelrate at least 1 step to stop.
    if(srd.decel_val == 0){
      srd.decel_val = -1;
    }

    // Find step to start decleration.
    srd.decel_start = step + srd.decel_val;

    // If the maximum speed is so low that we dont need to go via accelration state.
    if(srd.step_delay <= srd.min_delay)
	{
      srd.step_delay = srd.min_delay;
      srd.run_state = RUN;
    }
    else{
      srd.run_state = ACCEL;
    }

    // Reset counter.
    srd.accel_count = 0;
    MotionStatus = 1;
    //OCR1A = 10;
    AXIS_TIM_SetCompareN(AXIS_TIMx,100);
    TIM_SetAutoreload(AXIS_TIMx,100);
	  TIM_Cmd(AXIS_TIMx, ENABLE);
  }
}

void AXIS_TIMx_IRQHandler(void)//定时器中断处理
{ 
  // Holds next delay period.
  uint16_t new_step_delay;
  // Remember the last step delay used when accelrating.
  static uint16_t last_accel_delay;
  // Counting steps when moving.
  static uint32_t step_count = 0;
  // Keep track of remainder from new_step-delay calculation to incrase accurancy
  static int32_t rest = 0;
  static uint8_t i=0;

  if (TIM_GetITStatus(AXIS_TIMx, TIM_IT_CC1) != RESET)
  {
    /* Clear AXIS_TIMx Capture Compare1 interrupt pending bit*/
    TIM_ClearITPendingBit(AXIS_TIMx, TIM_IT_CC1);
    TIM_ClearFlag(AXIS_TIMx,TIM_FLAG_CC1);
  }
  AXIS_TIM_SetCompareN(AXIS_TIMx,srd.step_delay);
  TIM_SetAutoreload(AXIS_TIMx,srd.step_delay);
  if(srd.run_state)
  {
	  if(GPIO_ReadOutputDataBit(AxisPulsePort, AXIS_PULSE_1))
	  {
		  GPIO_ResetBits(AxisPulsePort, AXIS_PULSE_1);
		}
	  else
	  {
			GPIO_SetBits(AxisPulsePort, AXIS_PULSE_1);
	  }
	}
  i++;
  if(i==2)
  {
	  i=0;
    switch(srd.run_state) 
    {
      case STOP:
        step_count = 0;
        rest = 0;
        TIM_Cmd(AXIS_TIMx, DISABLE);			
        MotionStatus = 0;
        bEmgStopping=FALSE;
        break;

      case ACCEL:
        step_count++;
        if(srd.dir==CW)
        {	  	
          position++;
        }
        else
        {
          position--;
        }
        srd.accel_count++;
        new_step_delay = srd.step_delay - (((2 * (long)srd.step_delay) + rest)/(4 * srd.accel_count + 1));
        rest = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);
        if(step_count >= srd.decel_start  || bStopCmd || (bLmtPos && srd.dir==CW) || (bLmtNeg && srd.dir==CCW))
        {
          if(bStopCmd)
          {
            bStopCmd=FALSE;
            srd.accel_count = T1_FREQ/2/stpdecel*T1_FREQ/srd.step_delay/srd.step_delay*-1;
          }
          else if((bLmtPos && srd.dir==CW) || (bLmtNeg && srd.dir==CCW))
          {
            srd.accel_count = T1_FREQ/2/stpdecel*T1_FREQ/srd.step_delay/srd.step_delay*-1;
            bEmgStopping=TRUE;
          }
          else
          {
            srd.accel_count = srd.decel_val;
          }
          
          srd.run_state = DECEL;
        }
        else if(new_step_delay <= srd.min_delay)
        {
          last_accel_delay = new_step_delay;
          new_step_delay = srd.min_delay;
          rest = 0;
          srd.run_state = RUN;
        }
        break;

      case RUN:
        step_count++;
        if(srd.dir==CW)
        {	  	
          position++;
        }
        else
        {
          position--;
        }
        new_step_delay = srd.min_delay;
        if(step_count >= srd.decel_start || bStopCmd || (bLmtPos && srd.dir==CW) || (bLmtNeg && srd.dir==CCW))
        {
          if(bStopCmd)
          {
            bStopCmd=FALSE;
            srd.accel_count = T1_FREQ/2/stpdecel*T1_FREQ/srd.step_delay/srd.step_delay*-1;
          }
          else if((bLmtPos && srd.dir==CW) || (bLmtNeg && srd.dir==CCW))
          {
            srd.accel_count = T1_FREQ/2/stpdecel*T1_FREQ/srd.step_delay/srd.step_delay*-1;
            bEmgStopping=TRUE;
          }
          else
          {
            srd.accel_count = srd.decel_val;
          }
          // Start decelration with same delay as accel ended with.
          new_step_delay = last_accel_delay;
          srd.run_state = DECEL;
        }
        break;

      case DECEL:
        step_count++;
        if(srd.dir==CW)
        {	  	
          position++;
        }
        else
        {
          position--;
        }
        srd.accel_count++;
        new_step_delay = srd.step_delay - (((2 * (long)srd.step_delay) + rest)/(4 * srd.accel_count + 1));
        rest = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);
         if(bStopCmd)
        {
          bStopCmd=FALSE;
          srd.accel_count = T1_FREQ/2/stpdecel*T1_FREQ/srd.step_delay/srd.step_delay*-1;
        }
        else if(!bEmgStopping && ((bLmtPos && srd.dir==CW) || (bLmtNeg && srd.dir==CCW)))
        {
          srd.accel_count = T1_FREQ/2/stpdecel*T1_FREQ/srd.step_delay/srd.step_delay*-1;
          bEmgStopping=TRUE;
        }
        if(srd.accel_count >= 0)
        {
          srd.run_state = STOP;
        }
        break;
    }
    srd.step_delay = new_step_delay;
  }
}


void AxisEXTI_IRQHANDLER(void)//回零开关触发，记录触发时的位置值
{
  if(EXTI_GetITStatus(AxisEXTILine) == SET)
  {
    HomePos=position;  // 记录零点位置
		bZeroCapture=TRUE; // 寻找到零点
		if(MotionStatus)   // 运动中
		{
			bStopCmd=TRUE;   // 停止命令
		}
    EXTI_ClearITPendingBit(AxisEXTILine);
		DisableHomeCapture();
  }
}


/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
