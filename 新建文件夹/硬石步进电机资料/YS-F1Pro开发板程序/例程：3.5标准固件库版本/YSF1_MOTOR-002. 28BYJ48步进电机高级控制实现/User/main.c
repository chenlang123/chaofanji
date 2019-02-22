/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2015-10-04
  * ��    ��: ʹ�ñ�׼�ⷽ������LED������
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f10x.h"
#include "bsp/motor/bsp_uln2003.h"
#include "bsp/systick/bsp_SysTick.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
#define STEPMOTOR_CIRCLE_NUMBER             5   //  ת��Ȧ��
#define STEPMOTOR_DIRECTION                 1   // 1��˳ʱ��  0����ʱ��
#define STEPMOTOR_SPEED                     10  // �ٶȣ���ֵԽС���ٶ�Խ�죬��С����С��10


/* ˽�б��� ------------------------------------------------------------------*/
// �ٶȣ���ֵԽС���ٶ�Խ�죬��С����С��10
uint8_t speed=STEPMOTOR_SPEED;
// ת��Ȧ����28BYJ-48��������Ĳ���Ƕ�Ϊ5.625/64����ÿ64������ת5.625��
// ҪתһȦ��Ҫ360/5.625*64=4096�����塣
uint32_t Circle_number=STEPMOTOR_CIRCLE_NUMBER;

// 8�����Ŀ��ƣ�A->AB->B->BC->C->CD->D->DA
uint16_t step_motor[8]={0xFC7F,0xFCFF,0xFCBF,0xFDBF,0xFD3F,0xFF3F,0xFE3F,0xFE7F};


/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ���һ�����ݸ�ULN2003�Ӷ�ʵ���򲽽��������һ������
  * �������: step�����кţ�ѡ��step_motor�����Ӧ������
  *           direction������ѡ��
  *               ��ѡֵ��0��˳ʱ��
  *                       1����ʱ��
  * �� �� ֵ: ��
  * ˵    ��: ��
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
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
int main(void)
{
  ULN2003_GPIO_Init();
  
  SysTick_Init();
  
  /* ����ѭ�� */
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
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/

