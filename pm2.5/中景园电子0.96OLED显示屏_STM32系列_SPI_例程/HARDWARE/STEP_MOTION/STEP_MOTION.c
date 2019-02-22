//���������ص������ļ�


#include "STEP_MOTION.h"



extern int MODE_STATUS;
extern int WORK_STATUS;

int step_to_run;												//���еĲ���
unsigned short period[ACCELERATED_SPEED_LENGTH];//����洢���ٹ�����ÿһ����ʱ�����Զ�װ��ֵ
float fre[ACCELERATED_SPEED_LENGTH];		//  ����洢���ٹ��̵���ÿһ����Ƶ��

int MOTION_STATUS=0;

void CalculateSModelLine(float fre[], unsigned short period[], float len, float fre_max, float fre_min, float flexible)
{
		int i=0;
		float deno ;
		float melo ;
		float delt = fre_max-fre_min;
		for(; i<len; i++)
		{
				melo = flexible * (i-len/2) / (len/2);
				deno = 1.0 / (1 + expf(-melo));
				fre[i] = delt * deno + fre_min;
				period[i] = (unsigned short)(1000000.0 / fre[i]);
		}
		return ;
}
void STEP_MOTION_CHANGE(unsigned short *period_list,char mode)
{
		static int i;
		if(1== mode)
		{
			if(i<(ACCELERATED_SPEED_LENGTH-2))
			{
				i++;
				TIM1->ARR= period_list[i];
				TIM1->CCR3= period_list[i]/2;
//				TIM1->CCR2= period_list[i]/2;
			}
			else
			{
				i=0;
				MOTION_STATUS=2;
			}

		}
		else
		{
			if(i<(ACCELERATED_SPEED_LENGTH-2))
			{
				i++;
				TIM1->ARR= period_list[ACCELERATED_SPEED_LENGTH-i];
				TIM1->CCR3= period_list[ACCELERATED_SPEED_LENGTH-i]/2;
//				TIM1->CCR2= period_list[ACCELERATED_SPEED_LENGTH-i]/2;
			}
			else
			{
				i=0;
				MOTION_STATUS=0;
			}

		}
}
void TIM1_MOTION_WORK()
{
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
		{
			/* Clear AXIS_TIMx Capture Compare1 interrupt pending bit*/
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
			TIM_ClearFlag(TIM1,TIM_IT_Update);
			if((1==WORK_STATUS)&&(1==MOTION_STATUS))
			{
				if((0x01==MODE_STATUS))
				{
					STEP_MOTION_CHANGE((unsigned short *)&period,1);
				}
				else if((0x00==MODE_STATUS))
				{
					STEP_MOTION_CHANGE((unsigned short *)&period,1);
				}
				else
				{
					STEP_MOTION_CHANGE((unsigned short *)&period,1);
				}
			}
			if((1==WORK_STATUS)&&(2==MOTION_STATUS))
			{
			}
			if((0==WORK_STATUS)&&(2==MOTION_STATUS))
			{
				if((0x01==MODE_STATUS))
				{
					STEP_MOTION_CHANGE((unsigned short *)&period,0);
				}
				else if((0x00==MODE_STATUS))
				{
					STEP_MOTION_CHANGE((unsigned short *)&period,0);
				}
				else
				{
					STEP_MOTION_CHANGE((unsigned short *)&period,0);
				}
			}
			if((0==WORK_STATUS)&&(0==MOTION_STATUS))
			{
				 TIM_CtrlPWMOutputs(TIM1, DISABLE); 
			}
		}
}

void STEP_MOTION_IRQHandler(char tim_class)
{
	switch (tim_class)
	{
		case 1:
			TIM1_MOTION_WORK();
			break;
		default:
			break;
	}
}

