
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
extern u16 CCR3_Val;//占空比，周期为1000

extern int step_to_run;												//运行的步数
extern unsigned short period[ACCELERATED_SPEED_LENGTH];//数组存储加速过程中每一步定时器的自动装载值
extern float fre[ACCELERATED_SPEED_LENGTH];		//  数组存储加速过程的中每一步的频率




 int main(void)
{	
		/* 配置SysTick 为10us中断一次 */
	SysTick_Init();	    	 //延时函数初始化	  
	CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH,100000,6000,4);
	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级 	LED_Init();			     //LED端口初始化
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
 
