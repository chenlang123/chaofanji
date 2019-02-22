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
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	

//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  




void uart_init(u32 bound)
{
			//GPIO�˿�����
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//ʹ��USART1��GPIOAʱ���Լ����ù���ʱ��
#ifdef UART1
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ���Լ����ù���ʱ��
		//USART1_TX   PA.9
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
		GPIO_Init(GPIOA, &GPIO_InitStructure);
			//USART1_RX	  PA.10
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
		GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif	

#ifdef UART2
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//ʹ��USART2
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			//USART2_TX   PA.2
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.9
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		 
		//USART2_RX	  PA.3
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
		GPIO_Init(GPIOA, &GPIO_InitStructure);  

#endif	
	
#ifdef UART3

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //ʹ��UART3����GPIOB��ʱ�� 
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

		 //Usart1 NVIC ����

		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
		
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
		
		 //USART ��ʼ������

		USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

		USART_Init(USART1, &USART_InitStructure); //��ʼ������
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�����ж�
		USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ��� 
		
		USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

		USART_Init(USART2, &USART_InitStructure); //��ʼ������
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�
		USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ��� 

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
   //���-������������Ҫ�ȶ�SR,�ٶ�DR�Ĵ����������������жϵ�����
	if(USART_GetFlagStatus(USART1,USART_FLAG_ORE) == SET)
	{
			USART_ReceiveData(USART1);
			USART_ClearFlag(USART1,USART_FLAG_ORE);
	}
	 USART_ClearFlag(USART1,USART_IT_RXNE); //һ��Ҫ��������ж�
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


