/**
  ******************************************************************************
  * 文件名程: main.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2015-10-04
  * 功    能: 使用标准库方法控制LED灯亮灭
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
#include "stm32f10x.h"
#include "bsp/motor/bsp_uln2003.h"
#include "bsp/systick/bsp_SysTick.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
#define STEPMOTOR_CIRCLE_NUMBER             5   //  转动圈数
#define STEPMOTOR_DIRECTION                 1   // 1：顺时针  0：逆时针
#define STEPMOTOR_SPEED                     10  // 速度，该值越小，速度越快，最小不能小于10


/* 私有变量 ------------------------------------------------------------------*/
// 速度，该值越小，速度越快，最小不能小于10
uint8_t speed=STEPMOTOR_SPEED;
// 转动圈数：28BYJ-48步进电机的步距角度为5.625/64，即每64个脉冲转5.625度
// 要转一圈需要360/5.625*64=4096个脉冲。
uint32_t Circle_number=STEPMOTOR_CIRCLE_NUMBER;

// 8个节拍控制：A->AB->B->BC->C->CD->D->DA
uint16_t step_motor[8]={0xFC7F,0xFCFF,0xFCBF,0xFDBF,0xFD3F,0xFF3F,0xFE3F,0xFE7F};


/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 输出一个数据给ULN2003从而实现向步进电机发送一个脉冲
  * 输入参数: step：序列号，选择step_motor数组对应的数据
  *           direction：方向选择
  *               可选值：0：顺时针
  *                       1：逆时针
  * 返 回 值: 无
  * 说    明: 无
  */
static void step_motor_pulse(uint8_t step,uint8_t direction)
{
#if 1  
  uint16_t data;
  
  data=GPIO_ReadOutputData(ULN2003_GPIO)&0xFC3F;
  if(direction==1)
  {
    GPIO_Write(ULN2003_GPIO,data|step_motor[step]);
  }
  else
  {
    GPIO_Write(ULN2003_GPIO,data|step_motor[7-step]);
  }
#else
  uint8_t temp=step;
  
  if(direction==0)
  {
    temp=8-step;
  }
  switch(temp)
  {
    case 0:
      A_ON();B_OFF();C_OFF();D_OFF();
      break;
    case 1:
      A_ON();B_ON();C_OFF();D_OFF();
      break;
    case 2:
      A_OFF();B_ON();C_OFF();D_OFF();
      break;
    case 3:
      A_OFF();B_ON();C_ON();D_OFF();
      break;
    case 4:
      A_OFF();B_OFF();C_ON();D_OFF();
      break;
    case 5:
      A_OFF();B_OFF();C_ON();D_ON();
      break;
    case 6:
      A_OFF();B_OFF();C_OFF();D_ON();
      break;
    case 7:
      A_ON();B_OFF();C_OFF();D_ON();
      break;
  }  
#endif
}

/**
  * 函数功能: 主函数.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
int main(void)
{
  ULN2003_GPIO_Init();
  
  SysTick_Init();
  
  /* 无限循环 */
  while (1)
  {
     
  }
}


/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  static uint8_t count=0,step=0;
  static uint16_t pulse_count=0;
  
  if(Circle_number)
  {
    count++;    
    if(count==speed)
    {
      step_motor_pulse(step,STEPMOTOR_DIRECTION);
      step++;    
      pulse_count++;      
      if(step==8) step=0;
      count=0;      
    }
    
    if(pulse_count==4096)
    {
      pulse_count=0;
      Circle_number--;
    }
  }
}
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/

