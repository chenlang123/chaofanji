#ifndef __SYS_H
#define __SYS_H	
#include "stm32f10x.h"
																	    
	 
//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����



void NVIC_Configuration(void);



//���Ŷ���
#define KEY_START_GPIO				GPIOA
#define KEY_START_PIn					GPIO_Pin_5
#define KEY_STOP_GPIO					GPIOA
#define KEY_STOP_PIn					GPIO_Pin_6

#define MODE_STATUS_GPIO			GPIOA
#define MODE_STATUS_PIn1			GPIO_Pin_7
#define MODE_STATUS_PIn2			GPIO_Pin_8

#define STEP_MOTION_GPIO			GPIOA
#define STEP_MOTION_PWM2			GPIO_Pin_9
#define STEP_MOTION_PWM2N			GPIO_Pin_10

#define STEP_MOTION_GPIO2			GPIOB
#define STEP_MOTION_PWM3			GPIO_Pin_14
#define STEP_MOTION_PWM3N			GPIO_Pin_15

#define STEP_MOTION_ENA_GPIO	GPIOB
#define STEP_MOTION_ENA				GPIO_Pin_13
#define STEP_MOTION_PUL				GPIO_Pin_12

#define UART1_GPIO						GPIOA
#define UART1_TX							GPIO_Pin_9
#define UART1_RX							GPIO_Pin_10

#define UART2_GPIO						GPIOA
#define UART2_TX							GPIO_Pin_2
#define UART2_RX							GPIO_Pin_3

#define UART3_GPIO						GPIOB
#define UART3_TX							GPIO_Pin_10
#define UART3_RX							GPIO_Pin_11

#define UART4_GPIO						GPIOC
#define UART4_TX							GPIO_Pin_10
#define UART4_RX							GPIO_Pin_11

#define TM1637_CLK_GPIO				GPIOA
#define TM1637_CLK_Pin				GPIO_Pin_4

#define TM1637_DIO_GPIO				GPIOB
#define TM1637_DIO_Pin				GPIO_Pin_10

//���ܶ���
#define UART1
#define UART2


#endif
