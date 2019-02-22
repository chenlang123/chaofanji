#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "TM1637.h"

static __IO u32 TimingDelay;

int time_10us=0;
int time_1ms=0;
int time_10ms=0;
int time_1s=0;
int time_imint=0;
extern void KEY_scan(void);
extern char A[];
/*
 * ��������SysTick_Init
 * ����  ������ϵͳ�δ�ʱ�� SysTick
 * ����  ����
 * ���  ����
 * ����  ���ⲿ���� 
 */
void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms�ж�һ��
	 * SystemFrequency / 100000	 10us�ж�һ��
	 * SystemFrequency / 1000000 1us�ж�һ��
	 */
//	if (SysTick_Config(SystemFrequency / 100000))	// ST3.0.0��汾
	if (SysTick_Config(SystemCoreClock / 100000))	// ST3.5.0��汾
	{ 
		/* Capture error */ 
		while (1);
	}
		// �رյδ�ʱ��  
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
}


/*
 * ��������Delay_us
 * ����  ��us��ʱ����,10usΪһ����λ
 * ����  ��- nTime
 * ���  ����
 * ����  ��Delay_us( 1 ) ��ʵ�ֵ���ʱΪ 1 * 10us = 10us
 *       ���ⲿ���� 
 */

void delay_us(__IO u32 nTime)
{ 
	TimingDelay = nTime;	

	// ʹ�ܵδ�ʱ��  
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;

	while(TimingDelay != 0);
}

void delay_ms(__IO u32 nTime)
{ 
	TimingDelay = nTime*1000;	

	// ʹ�ܵδ�ʱ��  
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;

	while(TimingDelay != 0);
}

void TIME_10ms()
{
	KEY_scan();
}
void TIME_1s()
{
		static char flag=0;
		if (0==flag)
		{
			TM1637_NixieTubeDisplayChar(0,2);
			flag=1;
		}
		else
		{
			flag=0;
			TM1637_NixieTubeDisplay_off();
		}
		UART_SendString(USART1,A);
	//��ʾ״̬
}

/*
 * ��������TimingDelay_Decrement
 * ����  ����ȡ���ĳ���
 * ����  ����
 * ���  ����
 * ����  ���� SysTick �жϺ��� SysTick_Handler()����
 */  
void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
	{ 
	TimingDelay--;
	}

	time_10us++;
	if(time_10us>100)
	{
		time_10us=0;
		time_1ms++;
		time_10ms++;
	}
	if(time_10ms>10)
	{
		time_10ms=0;
		TIME_10ms();
	}
	if(time_1ms>1000)
	{
		time_1ms =0;
		time_1s++;
		TIME_1s();
	}
	if(time_1s>60)
	{
		time_1s=0;
		time_imint++;
		
	}
	if(time_imint>60)
	{
		time_imint=0;
	}
}

