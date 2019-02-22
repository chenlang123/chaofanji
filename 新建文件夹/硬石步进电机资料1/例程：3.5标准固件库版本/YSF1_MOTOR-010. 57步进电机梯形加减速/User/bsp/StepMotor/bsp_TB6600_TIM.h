#ifndef __TB6600_TIM_H__
#define __TB6600_TIM_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include <stm32f10x.h>

/* ���Ͷ��� ------------------------------------------------------------------*/
typedef enum {FALSE = 0, TRUE = !FALSE} bool;

enum // ����״̬
{
 	IDEL=0,       // ����״̬
 	FASTSEEK,     // ����Ѱ��״̬
	FASTSEEKSTOP, // ����Ѱ��ֹͣ״̬
	FASTSEEKBACK, // ��ʼ����Ѱ��״̬
	SLOWSEEK,     // ����Ѱ��
	SLOWSEEKSTOP, // ����Ѱ��ֹͣ״̬
	MOVETOZERO    // �ѻ���
};

typedef struct {
  uint8_t run_state ;//! What part of the speed ramp we are in. ��ת״̬
  uint8_t dir ;//! Direction stepper motor should move. ��ת����
  int32_t step_delay;//! Peroid of next timer delay. At start this value set the accelration rate. 
  uint32_t decel_start;//! What step_pos to start decelaration ��������
  int32_t decel_val;//! Sets deceleration rate. ���ü�������
  int32_t min_delay;//! Minimum time delay (max speed)��С��ʱʱ��(��Ӧ����ٶ�)
  int32_t accel_count;//! Counter used when accelerateing/decelerateing to calculate step_delay.�Ӽ��ٽ׶μ���ֵ
}speedRampData;

/* �궨�� --------------------------------------------------------------------*/
#define AXIS_TIMX_CLOCKCMD           RCC_APB2PeriphClockCmd
#define AXIS_TIMx_RCC_CLK            RCC_APB2Periph_TIM1
#define AXIS_TIMx                    TIM1
#define AXIS_TIMx_IRQn               TIM1_CC_IRQn
#define AXIS_TIMx_IRQHandler         TIM1_CC_IRQHandler
#define AXIS_TIMx_OCnInit            TIM_OC1Init
#define AXIS_TIM_SetCompareN         TIM_SetCompare1

#define AXIS_PULSE_CLK               RCC_APB2Periph_GPIOA
#define AXIS_PULSE_1                 GPIO_Pin_8        // �����������
#define AxisPulsePort                GPIOA

#define AXIS_DIR_CLK                 RCC_APB2Periph_GPIOB
#define AXIS_DIR_1                   GPIO_Pin_13        // �����������
#define AxisDirPort                  GPIOB
#define Axis_SETDIR_CW()             GPIO_SetBits(AxisDirPort, AXIS_DIR_1)
#define Axis_SETDIR_CCW()            GPIO_ResetBits(AxisDirPort, AXIS_DIR_1)

#define AXIS_LMTPOS_CLK              RCC_APB2Periph_GPIOC
#define AXIS_LMTPOS_1                GPIO_Pin_2        // ��ת��λ��������
#define AxisLmtPosPort               GPIOC

#define AXIS_LMTNEG_CLK              RCC_APB2Periph_GPIOC
#define AXIS_LMTNEG_1                GPIO_Pin_3        // ��ת��λ��������
#define AxisLmtNegPort               GPIOC

#define AXIS_HOME_CLK                RCC_APB2Periph_GPIOC
#define AXIS_HOME_1                  GPIO_Pin_4        // �����������
#define AxisHomePort                 GPIOC
#define AxisEXTI_PORTSource          GPIO_PortSourceGPIOC
#define AxisEXTIPinSource            GPIO_PinSource4
#define AxisEXTILine                 EXTI_Line4
#define AxisEXTI_IRQCHANNEL          EXTI4_IRQn
#define AxisEXTI_IRQHANDLER          EXTI4_IRQHandler

#define POSITIVE	                   1       // ����������
#define NEGATIVE	                   -1      // ���򣺷�����

#define CW                           0       // ��ת
#define CCW                          1       // ��ת
#define T1_FREQ                      10000   // Ƶ��

// Speed ramp states
#define STOP                         0      // ֹͣ
#define ACCEL                        1      // ����
#define DECEL                        2      // ����
#define RUN                          3      // ��������

/* ��չ���� ------------------------------------------------------------------*/
extern int32_t  ZeroDir;
extern uint32_t accel;         // ���ٶ�
extern uint32_t decel;         // ���ٶ�

/* �������� ------------------------------------------------------------------*/
void StepMotor_TIMx_Init(void);
void AxisMoveRel(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
void AxisMoveAbs(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
void LimitDetect(void);
void AxisHome(void);

#endif	/* __TB6600_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
