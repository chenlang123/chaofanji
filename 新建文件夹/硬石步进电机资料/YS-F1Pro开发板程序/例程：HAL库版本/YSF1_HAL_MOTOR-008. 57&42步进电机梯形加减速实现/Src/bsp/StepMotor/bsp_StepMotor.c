/**
  ******************************************************************************
  * 文件名程: bsp_STEPMOTOR.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.2
  * 编写日期: 2017-05-16
  * 功    能: 57&42型步进电机驱动器STEPMOTOR控制实现
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
#include "StepMotor/bsp_StepMotor.h"
#include <math.h>

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_STEPMOTOR;
speedRampData srd               = {STOP,CW,0,0,0,0,0};         // 加减速曲线变量
__IO int32_t  step_position     = 0;           // 当前位置
__IO uint8_t  MotionStatus      = 0;           //是否在运动？0：停止，1：运动

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: STEPMOTOR相关GPIO初始化配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
static void STEPMOTOR_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct; 
  
  /* 引脚端口时钟使能 */
  STEPMOTOR_TIM_GPIO_CLK_ENABLE();
  STEPMOTOR_DIR_GPIO_CLK_ENABLE();
  STEPMOTOR_ENA_GPIO_CLK_ENABLE();
  
  /* 步进电机驱动器：脉冲输出 */
  GPIO_InitStruct.Pin = STEPMOTOR_TIM_PUL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(STEPMOTOR_TIM_PUL_PORT, &GPIO_InitStruct);
  
  /* 步进电机驱动器：方向控制 */
  STEPMOTOR_DIR_FORWARD();
  GPIO_InitStruct.Pin = STEPMOTOR_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(STEPMOTOR_DIR_PORT, &GPIO_InitStruct);
  
  /* 步进电机驱动器：使能控制 */
  STEPMOTOR_OUTPUT_DISABLE();
  GPIO_InitStruct.Pin = STEPMOTOR_ENA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(STEPMOTOR_ENA_PORT, &GPIO_InitStruct);
}

/**
  * 函数功能: 步进电机驱动器定时器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void STEPMOTOR_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;             // 定时器时钟
  TIM_OC_InitTypeDef sConfigOC;                          // 定时器通道比较输出
  
  STEPMOTOR_TIM_RCC_CLK_ENABLE();
  
  /* STEPMOTOR相关GPIO初始化配置 */
  STEPMOTOR_GPIO_Init();
  
  /* 定时器基本环境配置 */
  htimx_STEPMOTOR.Instance = STEPMOTOR_TIMx;                          // 定时器编号
  htimx_STEPMOTOR.Init.Prescaler = STEPMOTOR_TIM_PRESCALER;           // 定时器预分频器
  htimx_STEPMOTOR.Init.CounterMode = TIM_COUNTERMODE_UP;              // 计数方向：向上计数
  htimx_STEPMOTOR.Init.Period = STEPMOTOR_TIM_PERIOD;                 // 定时器周期
  htimx_STEPMOTOR.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;          // 时钟分频
  HAL_TIM_Base_Init(&htimx_STEPMOTOR);

  /* 定时器时钟源配置 */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;       		// 使用内部时钟源
  HAL_TIM_ConfigClockSource(&htimx_STEPMOTOR, &sClockSourceConfig);

  /* 定时器比较输出配置 */
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;                // 比较输出模式：反转输出
  sConfigOC.Pulse = 0xFFFF;                      // 脉冲数
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;          // 输出极性
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;         // 互补通道输出极性
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // 快速模式
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;       // 空闲电平
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // 互补通道空闲电平
  HAL_TIM_OC_ConfigChannel(&htimx_STEPMOTOR, &sConfigOC, STEPMOTOR_TIM_CHANNEL_x);
  /* 使能比较输出通道 */
  TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_DISABLE);
  
  /* 配置定时器中断优先级并使能 */
  HAL_NVIC_SetPriority(STEPMOTOR_TIMx_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(STEPMOTOR_TIMx_IRQn);
  
  __HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR, STEPMOTOR_TIM_FLAG_CCx);
  /* 使能定时器比较输出 */
  __HAL_TIM_ENABLE_IT(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx);
  /* Enable the main output */
  __HAL_TIM_MOE_ENABLE(&htimx_STEPMOTOR);  
  HAL_TIM_Base_Start(&htimx_STEPMOTOR);// 使能定时器
}

/**
  * 函数功能: 相对位置运动：运动给定的步数
  * 输入参数: step：移动的步数 (正数为顺时针，负数为逆时针).
              accel  加速度,实际值为accel*0.1*rad/sec^2
              decel  减速度,实际值为decel*0.1*rad/sec^2
              speed  最大速度,实际值为speed*0.1*rad/sec
  * 返 回 值: 无
  * 说    明: 以给定的步数移动步进电机，先加速到最大速度，然后在合适位置开始
  *           减速至停止，使得整个运动距离为指定的步数。如果加减速阶段很短并且
  *           速度很慢，那还没达到最大速度就要开始减速
  */
void STEPMOTOR_AxisMoveRel(__IO int32_t step, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed)
{  
  __IO uint16_t tim_count;
  // 达到最大速度时的步数
  __IO uint32_t max_s_lim;
  // 必须要开始减速的步数（如果加速没有达到最大速度）
  __IO uint32_t accel_lim;

  if(step < 0) // 步数为负数
  {
    srd.dir = CCW; // 逆时针方向旋转
    STEPMOTOR_DIR_REVERSAL();
    step =-step;   // 获取步数绝对值
  }
  else
  {
    srd.dir = CW; // 顺时针方向旋转
    STEPMOTOR_DIR_FORWARD();
  }
  
  if(step == 1)    // 步数为1
  {
    srd.accel_count = -1;   // 只移动一步
    srd.run_state = DECEL;  // 减速状态.
    srd.step_delay = 1000;	// 短延时	
  }
  else if(step != 0)  // 如果目标运动步数不为0
  {
    // 我们的驱动器用户手册有详细的计算及推导过程

    // 设置最大速度极限, 计算得到min_delay用于定时器的计数器的值。
    // min_delay = (alpha / tt)/ w
    srd.min_delay = (int32_t)(A_T_x10/speed);

    // 通过计算第一个(c0) 的步进延时来设定加速度，其中accel单位为0.1rad/sec^2
    // step_delay = 1/tt * sqrt(2*alpha/accel)
    // step_delay = ( tfreq*0.676/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100
    srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);

    // 计算多少步之后达到最大速度的限制
    // max_s_lim = speed^2 / (2*alpha*accel)
    max_s_lim = (uint32_t)(speed*speed/(A_x200*accel/10));
    // 如果达到最大速度小于0.5步，我们将四舍五入为0
    // 但实际我们必须移动至少一步才能达到想要的速度
    if(max_s_lim == 0){
      max_s_lim = 1;
    }

    // 计算多少步之后我们必须开始减速
    // n1 = (n1+n2)decel / (accel + decel)
    accel_lim = (uint32_t)(step*decel/(accel+decel));
    // 我们必须加速至少1步才能才能开始减速.
    if(accel_lim == 0){
      accel_lim = 1;
    }

    // 使用限制条件我们可以计算出减速阶段步数
    if(accel_lim <= max_s_lim){
      srd.decel_val = accel_lim - step;
    }
    else{
      srd.decel_val = -(max_s_lim*accel/decel);
    }
    // 当只剩下一步我们必须减速
    if(srd.decel_val == 0){
      srd.decel_val = -1;
    }

    // 计算开始减速时的步数
    srd.decel_start = step + srd.decel_val;

    // 如果最大速度很慢，我们就不需要进行加速运动
    if(srd.step_delay <= srd.min_delay){
      srd.step_delay = srd.min_delay;
      srd.run_state = RUN;
    }
    else{
      srd.run_state = ACCEL;
    }    
    // 复位加速度计数值
    srd.accel_count = 0;
  }
  MotionStatus = 1; // 电机为运动状态
  tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
  __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+srd.step_delay); // 设置定时器比较值
  TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_ENABLE);// 使能定时器通道 
  STEPMOTOR_OUTPUT_ENABLE();
}

/**
  * 函数功能: 定时器中断服务函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 实现加减速过程
  */
void STEPMOTOR_TIMx_IRQHandler(void)//定时器中断处理
{ 
  __IO uint16_t tim_count=0;
  // 保存新（下）一个延时周期
  uint16_t new_step_delay=0;
  // 加速过程中最后一次延时（脉冲周期）.
  __IO static uint16_t last_accel_delay=0;
  // 总移动步数计数器
  __IO static uint32_t step_count = 0;
  // 记录new_step_delay中的余数，提高下一步计算的精度
  __IO static int32_t rest = 0;
  //定时器使用翻转模式，需要进入两次中断才输出一个完整脉冲
  __IO static uint8_t i=0;
  
  if(__HAL_TIM_GET_IT_SOURCE(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx) !=RESET)
  {
    // 清楚定时器中断
    __HAL_TIM_CLEAR_IT(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx);
    
    // 设置比较值
    tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
    __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+srd.step_delay);

    i++;     // 定时器中断次数计数值
    if(i==2) // 2次，说明已经输出一个完整脉冲
    {
      i=0;   // 清零定时器中断次数计数值
      switch(srd.run_state) // 加减速曲线阶段
      {
        case STOP:
          step_count = 0;  // 清零步数计数器
          rest = 0;        // 清零余值
          // 关闭通道
          TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_DISABLE);        
          __HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR, STEPMOTOR_TIM_FLAG_CCx);
          STEPMOTOR_OUTPUT_DISABLE();
          MotionStatus = 0;  //  电机为停止状态     
          break;

        case ACCEL:
          step_count++;      // 步数加1
          if(srd.dir==CW)
          {	  	
            step_position++; // 绝对位置加1
          }
          else
          {
            step_position--; // 绝对位置减1
          }
          srd.accel_count++; // 加速计数值加1
          new_step_delay = srd.step_delay - (((2 *srd.step_delay) + rest)/(4 * srd.accel_count + 1));//计算新(下)一步脉冲周期(时间间隔)
          rest = ((2 * srd.step_delay)+rest)%(4 * srd.accel_count + 1);// 计算余数，下次计算补上余数，减少误差
          if(step_count >= srd.decel_start)// 检查是够应该开始减速
          {
            srd.accel_count = srd.decel_val; // 加速计数值为减速阶段计数值的初始值
            srd.run_state = DECEL;           // 下个脉冲进入减速阶段
          }
          else if(new_step_delay <= srd.min_delay) // 检查是否到达期望的最大速度
          {
            last_accel_delay = new_step_delay; // 保存加速过程中最后一次延时（脉冲周期）
            new_step_delay = srd.min_delay;    // 使用min_delay（对应最大速度speed）
            rest = 0;                          // 清零余值
            srd.run_state = RUN;               // 设置为匀速运行状态
          }
          break;

        case RUN:
          step_count++;  // 步数加1
          if(srd.dir==CW)
          {	  	
            step_position++; // 绝对位置加1
          }
          else
          {
            step_position--; // 绝对位置减1
          }
          new_step_delay = srd.min_delay;     // 使用min_delay（对应最大速度speed）
          if(step_count >= srd.decel_start)   // 需要开始减速
          {
            srd.accel_count = srd.decel_val;  // 减速步数做为加速计数值
            new_step_delay = last_accel_delay;// 加阶段最后的延时做为减速阶段的起始延时(脉冲周期)
            srd.run_state = DECEL;            // 状态改变为减速
          }
          break;

        case DECEL:
          step_count++;  // 步数加1
          if(srd.dir==CW)
          {	  	
            step_position++; // 绝对位置加1
          }
          else
          {
            step_position--; // 绝对位置减1
          }
          srd.accel_count++;
          new_step_delay = srd.step_delay - (((2 * srd.step_delay) + rest)/(4 * srd.accel_count + 1)); //计算新(下)一步脉冲周期(时间间隔)
          rest = ((2 * srd.step_delay)+rest)%(4 * srd.accel_count + 1);// 计算余数，下次计算补上余数，减少误差
          
          //检查是否为最后一步
          if(srd.accel_count >= 0)
          {
            srd.run_state = STOP;
          }
          break;
      }      
      srd.step_delay = new_step_delay; // 为下个(新的)延时(脉冲周期)赋值
    }
  }
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
