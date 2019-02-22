#ifndef __BSP_ULN2003_H__
#define __BSP_ULN2003_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include <stm32f10x.h>

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
#define ULN2003_RCC_CLOCKCMD             RCC_APB2PeriphClockCmd
#define ULN2003_RCC_CLOCKGPIO            RCC_APB2Periph_GPIOG
#define ULN2003_GPIO_PIN                 (GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9)
#define ULN2003_GPIO                     GPIOG

#define A_ON()                           GPIO_SetBits(ULN2003_GPIO,GPIO_Pin_6)
#define A_OFF()                          GPIO_ResetBits(ULN2003_GPIO,GPIO_Pin_6)
#define B_ON()                           GPIO_SetBits(ULN2003_GPIO,GPIO_Pin_7)
#define B_OFF()                          GPIO_ResetBits(ULN2003_GPIO,GPIO_Pin_7)
#define C_ON()                           GPIO_SetBits(ULN2003_GPIO,GPIO_Pin_8)
#define C_OFF()                          GPIO_ResetBits(ULN2003_GPIO,GPIO_Pin_8)
#define D_ON()                           GPIO_SetBits(ULN2003_GPIO,GPIO_Pin_9)
#define D_OFF()                          GPIO_ResetBits(ULN2003_GPIO,GPIO_Pin_9)

/* ��չ���� ------------------------------------------------------------------*/
/* �������� ------------------------------------------------------------------*/
void ULN2003_GPIO_Init(void);

#endif  // __BSP_ULN2003_H__

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
