/*******************************************************************************
 *Filename:       sht30_dis.c
 *Revised:        $Date: 2017-06-15 15:23 $
 *Author:   	  double	
 *Description:    ????????
 *******************************************************************************
 *************************????**********************************************
 *******************************************************************************
 *                   
 *            --------------------
 *           |                PB7 |-->   SCL
 *           |                PB6 |-->   SDA
 *            --------------------
*******************************************************************************/

#include "SHT30.h"  


float TemValue = 0;
float RhValue = 0;
unsigned char buffer[6];


//IIC??????
//void IIC_Init(void);                                                           //???IIC?IO?				 
void SCL_L(void);
void SCL_H(void);
void SDA_L(void);
void SDA_H(void);
unsigned char SDA_Read(void);
void i2c_delay(void);
void i2c_noAck(void);
void i2c_ack(void);
void i2c_stop(void);
void i2c_delay(void);
unsigned char i2c_star(void);
unsigned char i2c_waitAck(void);
void i2c_sendByte( unsigned char byte );
unsigned char i2c_readByte(void);
void SHT3X_WriteCMD(unsigned int cmd);
void SHT3X_ReadState(unsigned char *temp);
void SHT3X_SetPeriodicMeasurement(void);
void SHX3X_ReadResults(unsigned int cmd,  unsigned char *p);
unsigned char SHT3X_CalcCrc(unsigned char *data, unsigned char nbrOfBytes);
unsigned char SHT3X_CheckCrc(unsigned char *pdata, unsigned char nbrOfBytes, unsigned char checksum);
float SHT3X_CalcTemperature(unsigned int rawValue);
float SHT3X_CalcRH(unsigned int rawValue);
void SHT_GetValue(void);
/*******************************************************************************
 * @fn        sht30_dis_init
 *
 * @brief    ???
 *
 * @return  none
 *
 ******************************************************************************/
 
 
 
 
 
 
 
 /*
 void delay_us(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}*/
void sht30_dis_init(void){
	
//  SCL_OUT();
	
	GPIO_InitTypeDef GPIO_InitStructure;

//  __HAL_RCC_GPIOB_CLK_ENABLE();
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;					//????
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_7); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;							//????,????????
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  delay_us(250);
  SHT3X_SetPeriodicMeasurement();//?????????
	
  delay_us(150); 
  SHT_GetValue();
  delay_us(150); 
}

void SDA_IN(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;							//????,????????
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}
void SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;					//????
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void SDA_H(void)
{
  SDA_OUT();
  IIC_SDA_H;
}
void SDA_L(void)
{
  SDA_OUT();
  IIC_SDA_L;
}
unsigned char SDA_Read(void)
{
  SDA_IN();
  return READ_SDA;
}
void SCL_H(void)
{
    IIC_SCL_H;
}
void SCL_L(void)
{
    IIC_SCL_L;
}


void i2c_delay(void)
{
//  unsigned int z;
//	
//	for(z=0;z<10;z++);
	//2M??,??????????????20ms?????
	__nop();
}


unsigned char i2c_star(void)
{
  SDA_H();
  SCL_H();
  i2c_delay();
  if (!SDA_Read())
    return 1;
  SDA_L();
  i2c_delay();
  if (SDA_Read())
    return 1;
  SDA_L();
  SCL_L();
  i2c_delay();
  return 0;
}

void i2c_stop(void)
{
  SCL_L();
  i2c_delay();
  SDA_L();
  i2c_delay();
  SCL_H();
  i2c_delay();
  SDA_H();
  i2c_delay();
}

void i2c_ack(void)
{
  SCL_L();
  i2c_delay();
  SDA_L();
  i2c_delay();
  SCL_H();
  i2c_delay();
  SCL_L();
  i2c_delay();
}

void i2c_noAck(void)
{
  SCL_L();
  i2c_delay();
  SDA_H();
  i2c_delay();
  SCL_H();
  i2c_delay();
  SCL_L();
  i2c_delay();
}

unsigned char i2c_waitAck(void)
{
  unsigned char t = 200;
  
  SCL_L(); 
  i2c_delay();
  SDA_H();
  i2c_delay();
  //////////////////////////?????,IO????????????,??????
  SDA_Read();
  //////////////////////////
  SCL_H();
  i2c_delay();    
  
  while( SDA_Read() )
  {
    t --;
    i2c_delay();

    if(t==0)
    {
       SCL_L();
        return 1;
    }
    i2c_delay();
  }
  i2c_delay();
  SCL_L();
  i2c_delay();
  return 0;
}

void i2c_sendByte( unsigned char byte )
{
  unsigned char i = 8;
  while (i--) {
    SCL_L();
    i2c_delay();
    if (byte & 0x80)
        SDA_H();
    else
        SDA_L();
    byte <<= 1;
    i2c_delay();
    SCL_H();
    i2c_delay();
  }
  SCL_L();
  i2c_delay();

}

unsigned char i2c_readByte(void)
{
  unsigned char i = 8;
  unsigned char byte = 0;

  SDA_H();
  SDA_Read();
  while (i--) 
  {
    byte <<= 1;
    SCL_L();
    i2c_delay();
    SCL_H();
    i2c_delay();
    if (SDA_Read()) 
    {
        byte |= 0x01;
    }
  }
  SCL_L();
  i2c_delay();

  return byte;
}

void SHT3X_WriteCMD(unsigned int cmd)
{
  i2c_star();
  i2c_sendByte(i2cAddWrite_8bit);
  i2c_waitAck();
  i2c_sendByte(cmd>>8);
  i2c_waitAck();
  i2c_sendByte(cmd);
  i2c_waitAck();
  i2c_stop();
}

void SHT3X_SetPeriodicMeasurement(void)
{
    SHT3X_WriteCMD(CMD_MEAS_PERI_2_H);
}
//??SHT30?????
void SHT3X_ReadState(unsigned char *temp)
{
    i2c_star();
    i2c_sendByte(i2cAddWrite_8bit);
    i2c_waitAck();
    i2c_sendByte(0xf3);
    i2c_waitAck();
    i2c_sendByte(0X2d);
    i2c_waitAck();
    
    i2c_star();
    i2c_sendByte(i2cAddRead_8bit);
    i2c_waitAck();

    temp[0] = i2c_readByte();//?
    i2c_ack();
    temp[1] = i2c_readByte();//?
    i2c_ack();
    temp[2] = i2c_readByte();//??
    i2c_noAck();
    
    i2c_stop(); 
    
}

//??SHT30??
void SHX3X_ReadResults(unsigned int cmd,  unsigned char *p)
{
  i2c_star();
  i2c_sendByte(i2cAddWrite_8bit);
  i2c_waitAck();
  i2c_sendByte(cmd>>8);
  i2c_waitAck();
  i2c_sendByte(cmd);
  i2c_waitAck();
  
  i2c_star();
  i2c_sendByte(i2cAddRead_8bit);
    
  if(i2c_waitAck()==0){     
    i2c_delay();
    i2c_delay();
    i2c_delay(); 
    
    p[0] = i2c_readByte();//???
    i2c_ack();
    p[1] = i2c_readByte();//???
    i2c_ack();
    p[2] = i2c_readByte();//??
    i2c_ack();
    p[3] = i2c_readByte();//???
    i2c_ack();
    p[4] = i2c_readByte();//???
    i2c_ack();
    p[5] = i2c_readByte();//??
    i2c_noAck();
    i2c_stop();
  }
}

//??
unsigned char SHT3X_CalcCrc(unsigned char *data, unsigned char nbrOfBytes)
{
	unsigned char bit;        // bit mask
    unsigned char crc = 0xFF; // calculated checksum
    unsigned char byteCtr;    // byte counter

    // calculates 8-Bit checksum with given polynomial
    for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
        crc ^= (data[byteCtr]);
        for(bit = 8; bit > 0; --bit) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ POLYNOMIAL;
            }  else {
                crc = (crc << 1);
            }
        }
    }
	return crc;
}
//????
unsigned char SHT3X_CheckCrc(unsigned char *pdata, unsigned char nbrOfBytes, unsigned char checksum)
{
    unsigned char crc;
	crc = SHT3X_CalcCrc(pdata, nbrOfBytes);// calculates 8-Bit checksum
    if(crc != checksum) 
    {   
        return 1;           
    }
    return 0;              
}
//????
float SHT3X_CalcTemperature(unsigned int rawValue)
{
    // calculate temperature 
    float temp;
    temp = (175 * (float)rawValue / 65535 - 45) ; // T = -45 + 175 * rawValue / (2^16-1)
    return temp;
}
//????
float SHT3X_CalcRH(unsigned int rawValue)
{
    // calculate relative humidity [%RH]
    float temp1 = (100 * (float)rawValue / 65535) ;  // RH = rawValue / (2^16-1) * 10

    return temp1;
}

//????---20ms??????
void SHT_GetValue(void)
{
    unsigned char temp = 0;
    unsigned int dat;
    unsigned char p[3];
    unsigned char cnt;
    unsigned char tem_status,hum_status;
    cnt = 0;
    tem_status = 0;
    hum_status = 0;
    
    while(cnt++<2){
      
      SHX3X_ReadResults(CMD_FETCH_DATA, buffer);//??????

      p[0] = buffer[0];
      p[1] = buffer[1];
      p[2] = buffer[2];
      temp = SHT3X_CheckCrc(p,2,p[2]);//??
     if( !temp ) 
      {
        dat = ((unsigned int)buffer[0] << 8) | buffer[1];
        TemValue = (float)SHT3X_CalcTemperature( dat )-TEM_CHEAK_VALUE;    //???????
        tem_status = 0;
      }else{
        tem_status = 1;  
      }

      p[0] = buffer[3];
      p[1] = buffer[4];
      p[2] = buffer[5];
      temp = SHT3X_CheckCrc(p,2,p[2]);//??
      if( !temp )
      {
          dat = ((unsigned int)p[0] << 8) | p[1];
          RhValue = SHT3X_CalcRH( dat )+RH_CHEAK_VALUE; //???????
          hum_status = 0;
      }else{
        hum_status = 1;  
        
      }
      
      if((tem_status==0) && (hum_status==0)){   
        break;
      }else{
//        SCL_OUT();
//        delay_user(250);
//        SHT3X_SetPeriodicMeasurement();//?????????
//        delay_user(150);
      }
      delay_us(10); 
    }
    if((tem_status==0) && (hum_status==0)){   
    }else{
//      SCL_OUT();
      delay_us(250);
      SHT3X_SetPeriodicMeasurement();//?????????
      delay_us(150);
    }
      
    
}
/*
*/

