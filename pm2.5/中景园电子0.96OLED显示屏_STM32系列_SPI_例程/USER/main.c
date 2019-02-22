
#include "delay.h"
#include "sys.h"
#include "oled.h"
#include "bmp.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "SHT30.h" 
#include "DS3231.h"
#include "TIM.h"
#include "key.h"
#include "STEP_MOTION.h"
#include <math.h>
#include "TM1637.h" 

int Flag_USART2=0;
int Flag_USART1=0;
int Flag_Bluetooth=0;

char A[]="ABCD";
extern char Rxbuffer1[USART_REC_LEN];
extern char Rxbuffer2[USART_REC_LEN];
extern char Txbuffer2[USART_REC_LEN];
void UART2_SendByte(u8 Byte);
void UART2_SendData(u8 *data,u8 len);
void UART2_SendString(char *CMD);
void UART1_SendByte(u8 Byte);
void UART1_SendData(u8 *data,u8 len);
void UART1_SendString(char *CMD);

extern u16 CCR2_Val;
extern u16 CCR3_Val;//ռ�ձȣ�����Ϊ1000

extern int step_to_run;												//���еĲ���
extern unsigned short period[ACCELERATED_SPEED_LENGTH];//����洢���ٹ�����ÿһ����ʱ�����Զ�װ��ֵ
extern float fre[ACCELERATED_SPEED_LENGTH];		//  ����洢���ٹ��̵���ÿһ����Ƶ��




 int main(void)
{	
		/* ����SysTick Ϊ10us�ж�һ�� */
	SysTick_Init();	    	 //��ʱ������ʼ��	  
	CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH,100000,6000,4);
	NVIC_Configuration(); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ� 	LED_Init();			     //LED�˿ڳ�ʼ��
	uart_init(9600);
	UART_SendString(USART2,A);
	TIM1_GPIO_Config();
	TIM1_Mode_Config(); 
	KEY_Config();
	TM1637_Init();
	 
	
	while(1) 
	{	
		delay_ms(50);
		
	}
}
 
