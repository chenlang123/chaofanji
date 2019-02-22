/******************************Copyright (c)***********************************
*                Jiangsu Zhihai Electronic Technology Co., Ltd.
*                      Research & Development Department
*
*													www.smarthealth-tech.com
*
-------------------------------------------------------------------------------
* @file    TM1637.h
* @author  GU DONGDONG
* @date    2015-11-25  
*******************************************************************************/
#ifndef TM1637_H
#define TM1637_H

#include "sys.h"

#define TM1637_VCC           PBout(11)  
#define TM1637_CLK           PAout(4)  
#define TM1637_DIO           PBout(10)  
#define TM1637_READ_DIO      PBin(10) 
  
//IO·½ÏòÉèÖÃ
#define TM1637_DIO_IN()     {GPIOB->CRH&=0XFFFFF0FF;GPIOB->CRH|=8<<8;}
#define TM1637_DIO_OUT()    {GPIOB->CRH&=0XFFFFF0FF;GPIOB->CRH|=3<<8;}
/** 
  * @brief  Configuration Dots's Mode enumeration  
  */
typedef enum
{ 
	DulDot = 0x00,
  LowDot= 0x01,
  HighDot= 0x2,
}DisDotSetting;
typedef enum
{ 
	sty = 0x00,
  Jep = 0x01, 
}DisDotMode;

//extern unsigned char NumDis[];
void TM1637_Delay_us(unsigned  int Nus); 
void TM1637_Start(void);
void TM1637_Ack(void);
void TM1637_Stop(void);
void TM1637_WriteByte(unsigned char oneByte);
unsigned char TM1637_ScanKey(void);
void TM1637_NixieTubeDisplay(void);
void TM1637_NixieTubeDisplayChar(unsigned char ch,unsigned char p);
void TM1637_NixieTubeDisplayNum(short Num,DisDotSetting Set,DisDotMode Mode);
void TM1637_Init(void);
void TM1637LED_Init(void);
void TM1637_NixieTubeDisplay_off(void);
#endif
