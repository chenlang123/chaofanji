#include "sys.h"
#include "usart.h"	  


 
char Rxbuffer1[USART_REC_LEN];
char Rxbuffer2[USART_REC_LEN];
char Txbuffer2[USART_REC_LEN];
volatile char Rxlen1 =0;
volatile char Rxlen2 =0;
extern int Flag_USART2;
extern int Flag_USART1;
extern int Flag_Bluetooth;
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	

//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  




void uart_init(u32 bound)
{
			//GPIO端口设置
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//使能USART1，GPIOA时钟以及复用功能时钟
#ifdef UART1
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟以及复用功能时钟
		//USART1_TX   PA.9
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
		GPIO_Init(GPIOA, &GPIO_InitStructure);
			//USART1_RX	  PA.10
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
		GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif	

#ifdef UART2
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//使能USART2
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			//USART2_TX   PA.2
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.9
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		 
		//USART2_RX	  PA.3
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
		GPIO_Init(GPIOA, &GPIO_InitStructure);  

#endif	
	
#ifdef UART3

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //使能UART3所在GPIOB的时钟 
					// Configure USART3 Tx (PB.10) as alternate function push-pull  
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
		GPIO_Init(GPIOB, &GPIO_InitStructure); 

		// Configure USART3 Rx (PB.11) as input floating    
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;  
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
		GPIO_Init(GPIOB, &GPIO_InitStructure);  
#endif	
#ifdef UART4

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);//for UART4
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
			 
		// Configure USART Tx as alternate function push-pull 
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		// Configure USART Rx as input floating 
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif			

		 //Usart1 NVIC 配置

		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
		
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
		
		 //USART 初始化设置

		USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

		USART_Init(USART1, &USART_InitStructure); //初始化串口
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
		USART_Cmd(USART1, ENABLE);                    //使能串口 
		
		USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

		USART_Init(USART2, &USART_InitStructure); //初始化串口
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断
		USART_Cmd(USART2, ENABLE);                    //使能串口 

}

void USART1_IRQHandler(void)
{
	u8 ch;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{ 	
			if(Rxlen1 >= (sizeof(Rxbuffer1)-1))
			{
				Rxlen1 = 0;
			}
	    ch = USART1->DR;
			Rxbuffer1[Rxlen1] = USART_ReceiveData(USART1);
			if(Rxbuffer1[Rxlen1] == 'R')
			{
				Rxlen1 = 0;
			}
			if(Rxbuffer1[Rxlen1] == 0x0a)
			{
				Flag_USART1 = 1;
				Rxbuffer1[Rxlen1] = 0;
			}
			Rxlen1 ++;
	}
   //溢出-如果发生溢出需要先读SR,再读DR寄存器则可清除不断入中断的问题
	if(USART_GetFlagStatus(USART1,USART_FLAG_ORE) == SET)
	{
			USART_ReceiveData(USART1);
			USART_ClearFlag(USART1,USART_FLAG_ORE);
	}
	 USART_ClearFlag(USART1,USART_IT_RXNE); //一定要清除接收中断
}
void USART2_IRQHandler(void)
{
	u8 ch;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{ 	
			if(Rxlen2 >= 35)
			{
				Rxlen2 = 0;
			}
	    ch = USART2->DR;
			Rxbuffer2[Rxlen2] = USART_ReceiveData(USART2);
			if(Rxbuffer2[Rxlen2] == 0x42)
			{
				Rxlen2 =0;
			}
			++Rxlen2;
			if(Rxbuffer2[28] == 0x91)
			{
				Flag_USART1 = 1;
				Rxbuffer2[33]='\0';
			}
	}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
}

void UART_SendByte(USART_TypeDef* USARTx,u8 Byte)
{
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);	
	USART_SendData(USARTx, Byte);
}
void UART_SendString(USART_TypeDef* USARTx,char *CMD)
{
	while(*CMD!=0)
	{
		UART_SendByte(USARTx,*CMD);
		CMD++;
	}
}

void UART_SendData(USART_TypeDef* USARTx,u8 *data,u8 len)
{
	u8 i;
	for(i=0;i<len;i++)
	{
		UART_SendByte(USARTx,data[i]);
	}
}


