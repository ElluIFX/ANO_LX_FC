#ifndef _PWM_OUT_H_
#define _PWM_OUT_H_

#include "SysConfig.h"

//ע�⣬����ESCУ׼���ܣ������ܷ�������Ԥ�ϵ��𻵻��������˺�������Ը���
//һ����ҪУ׼ʱ���������������������ⷢ�����⡣
//У׼ESC�ɹ��󣬼ǵùرմ˹��ܣ�����������⡣
#define ESC_CALI 0 //1���򿪣�0���رա�

//#define PWM_FRE_HZ        400
//#define PWM_FRE_HZ        350    //�����ߵ�����Գ����� 350 hz

void DrvPwmOutInit(void);
void DrvMotorPWMSet(int16_t pwm[]); //��Χ0-1000

#endif
