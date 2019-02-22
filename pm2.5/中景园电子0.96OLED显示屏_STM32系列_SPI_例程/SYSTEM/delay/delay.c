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
 * 函数名：SysTick_Init
 * 描述  ：启动系统滴答定时器 SysTick
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用 
 */
void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms中断一次
	 * SystemFrequency / 100000	 10us中断一次
	 * SystemFrequency / 1000000 1us中断一次
	 */
//	if (SysTick_Config(SystemFrequency / 100000))	// ST3.0.0库版本
	if (SysTick_Config(SystemCoreClock / 100000))	// ST3.5.0库版本
	{ 
		/* Capture error */ 
		while (1);
	}
		// 关闭滴答定时器  
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
}


/*
 * 函数名：Delay_us
 * 描述  ：us延时程序,10us为一个单位
 * 输入  ：- nTime
 * 输出  ：无
 * 调用  ：Delay_us( 1 ) 则实现的延时为 1 * 10us = 10us
 *       ：外部调用 
 */

void delay_us(__IO u32 nTime)
{ 
	TimingDelay = nTime;	

	// 使能滴答定时器  
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;

	while(TimingDelay != 0);
}

void delay_ms(__IO u32 nTime)
{ 
	TimingDelay = nTime*1000;	

	// 使能滴答定时器  
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
	//显示状态
}

/*
 * 函数名：TimingDelay_Decrement
 * 描述  ：获取节拍程序
 * 输入  ：无
 * 输出  ：无
 * 调用  ：在 SysTick 中断函数 SysTick_Handler()调用
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

