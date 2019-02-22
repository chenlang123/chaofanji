/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2015-10-04
  * ��    ��: 57��������ٶȵ���(�������)
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
#include "bsp/57StepMotor/bsp_TB6600_TIM.h" 
#include "bsp/key/bsp_key.h"
#include "bsp/usart/bsp_debug_usart.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
uint8_t dir=0; // 0 ��˳ʱ��   1����ʱ�� 
uint8_t ena=0; // 0 ���������� 1��ͣ��

/* ��չ���� ------------------------------------------------------------------*/
extern __IO uint16_t prescaler[4]; /* ��������ٶȿ��ƣ��ɵ��ڷ�ΧΪ 650 -- 3500 ��ֵԽС�ٶ�Խ�� */

/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
int main(void)
{  
  uint8_t key1_count=1;
  /* ��ʼ������ */
  KEY_GPIO_Init();
  
  DEBUG_USART_Init();
  printf("4�Ჽ��������ƣ�KEY1��KEY2��ϵ��ڵ��ת��\n");
  printf("key1_count:%d\n",key1_count);
  
  /* ��ʼ����ʱ��PWM��� */
  TB6600_TIMx_PWM_Init();    
  
  /* ����ѭ�� */
  while (1)    
  {
    if(KEY1_StateRead()==KEY_DOWN)  // ����ѡ��
    {
       key1_count++;
       if(key1_count>10)
         key1_count=1;
       printf("key1_count:%d\n",key1_count);
    }
    if(KEY2_StateRead()==KEY_DOWN)  // ���ܵ���
    {
      printf("key1_count:%d\n",key1_count);
      switch(key1_count)
      {
        case 1:
          prescaler[0]-=100;
          if(prescaler[0]<700)  // ����ٶ�����
            prescaler[0]=700;
          printf("prescaler[0]:%d\n",prescaler[0]);
          break;
        case 2:
          prescaler[0]+=100;
          if(prescaler[0]>3500)         // �����ٶ�����
            prescaler[0]=3500;
          printf("prescaler[0]:%d\n",prescaler[0]);
          break;
        case 3:
          prescaler[1]-=100;
          if(prescaler[1]<700)  // ����ٶ�����
            prescaler[1]=700;
          printf("prescaler[1]:%d\n",prescaler[1]);
          break;
        case 4:
          prescaler[1]+=100;
          if(prescaler[1]>3500)         // �����ٶ�����
            prescaler[1]=3500;
          printf("prescaler[1]:%d\n",prescaler[1]);
          break;
        case 5:
          prescaler[2]-=100;
          if(prescaler[2]<700)  // ����ٶ�����
            prescaler[2]=700;
          printf("prescaler[2]:%d\n",prescaler[2]);
          break;
        case 6:
          prescaler[2]+=100;
          if(prescaler[2]>3500)         // �����ٶ�����
            prescaler[2]=3500;
          printf("prescaler[2]:%d\n",prescaler[2]);
          break;
        case 7:
          prescaler[3]-=100;
          if(prescaler[3]<700)  // ����ٶ�����
            prescaler[3]=700;
          printf("prescaler[3]:%d\n",prescaler[3]);
          break;
        case 8:
          prescaler[3]+=100;
          if(prescaler[3]>3500)         // �����ٶ�����
            prescaler[3]=3500;
          printf("prescaler[3]:%d\n",prescaler[3]);
          break;          
        case 9:                // �������
          printf("���ת������ı�\n");
          if(dir==0)
          {
            GPIO_ResetBits(TB6600_DIR1_PORT,TB6600_DIR1_PIN);  // ��ʱ��
            GPIO_ResetBits(TB6600_DIR2_PORT,TB6600_DIR2_PIN);  // ��ʱ��
            GPIO_ResetBits(TB6600_DIR3_PORT,TB6600_DIR3_PIN);  // ��ʱ��
            GPIO_ResetBits(TB6600_DIR4_PORT,TB6600_DIR4_PIN);  // ��ʱ��
            dir=1;
          }
          else
          {
            GPIO_SetBits(TB6600_DIR1_PORT,TB6600_DIR1_PIN);  // ˳ʱ��
            GPIO_SetBits(TB6600_DIR2_PORT,TB6600_DIR2_PIN);  // ˳ʱ��
            GPIO_SetBits(TB6600_DIR3_PORT,TB6600_DIR3_PIN);  // ˳ʱ��
            GPIO_SetBits(TB6600_DIR4_PORT,TB6600_DIR4_PIN);  // ˳ʱ��
            dir=0;
          }
          break;  
        case 10:                // ʹ�ܿ���
          printf("���ת��ʹ�ܿ���\n");
          if(ena==0)
          {
            GPIO_ResetBits(TB6600_ENA1_PORT,TB6600_ENA1_PIN); // ͣ��
            GPIO_ResetBits(TB6600_ENA2_PORT,TB6600_ENA2_PIN); // ͣ��
            GPIO_ResetBits(TB6600_ENA3_PORT,TB6600_ENA3_PIN); // ͣ��
            GPIO_ResetBits(TB6600_ENA4_PORT,TB6600_ENA4_PIN); // ͣ��
            ena=1;
            
          }
          else
          {
            GPIO_SetBits(TB6600_ENA1_PORT,TB6600_ENA1_PIN);  // ��������
            GPIO_SetBits(TB6600_ENA2_PORT,TB6600_ENA2_PIN);  // ��������
            GPIO_SetBits(TB6600_ENA3_PORT,TB6600_ENA3_PIN);  // ��������
            GPIO_SetBits(TB6600_ENA4_PORT,TB6600_ENA4_PIN);  // ��������
            ena=0;
          }
          break;
        default:
          break;          
      }
    }      
  }
}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
