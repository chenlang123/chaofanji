#include "key.h"
#include "sys.h"


extern int MODE_STATUS;
extern int WORK_STATUS;
extern int MOTION_STATUS;

void KEY_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  //GPIO_Pin_2 start GPIO_Pin_3 stop GPIO_Pin_4|GPIO_Pin_5 挡位  1档 01 2档 11 3档10
  GPIO_InitStructure.GPIO_Pin =  KEY_START_PIn | KEY_STOP_PIn|MODE_STATUS_PIn1|MODE_STATUS_PIn2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(KEY_START_GPIO, &GPIO_InitStructure);
}
void KEY_scan(void)
{
	static char start_status=0;
	static char start_last_status=0;
	static char stop_status=0;
	static char stop_last_status=0;
	
	if((1==start_last_status)&&(0==start_status)&&(0==GPIO_ReadInputDataBit(KEY_START_GPIO,KEY_START_PIn)))
	{

		WORK_STATUS=1;
		MOTION_STATUS=1;
		MODE_STATUS=GPIO_ReadInputDataBit(MODE_STATUS_GPIO,MODE_STATUS_PIn1)|GPIO_ReadInputDataBit(MODE_STATUS_GPIO,MODE_STATUS_PIn2);
		TIM_CtrlPWMOutputs(TIM1, ENABLE); //开启步进电机驱动
	}
	if((1==stop_last_status)&&(0==stop_status)&&(0==GPIO_ReadInputDataBit(KEY_STOP_GPIO,KEY_STOP_PIn)))
	{
		WORK_STATUS=0;
		
	}
	start_last_status=start_status;
	start_status=GPIO_ReadInputDataBit(KEY_START_GPIO,KEY_START_PIn);
	stop_last_status=stop_status;
	stop_status=GPIO_ReadInputDataBit(KEY_STOP_GPIO,KEY_STOP_PIn);
}

