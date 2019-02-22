#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);


void UART2_SendByte(u8 Byte);
void UART2_SendData(u8 *data,u8 len);
void UART2_SendString(char *CMD);

void UART_SendData(USART_TypeDef* USARTx,u8 *data,u8 len);
void UART_SendString(USART_TypeDef* USARTx,char *CMD);
void UART_SendByte(USART_TypeDef* USARTx,u8 Byte);
#endif


