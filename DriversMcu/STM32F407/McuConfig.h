#ifndef _MCUCONFIG_H_
#define _MCUCONFIG_H_
#include "stm32f4xx.h"

//=======================================
/***************�ж����ȼ�******************/
#define NVIC_GROUP NVIC_PriorityGroup_3 //�жϷ���ѡ��

#define NVIC_PWMIN_P 2 //���ջ��ɼ��ж�����
#define NVIC_PWMIN_S 1

#define NVIC_TIME_P 7 //��ʱ���ж�����
#define NVIC_TIME_S 1

#define NVIC_UART6_P 4 //����6�ж�����
#define NVIC_UART6_S 1

#define NVIC_UART5_P 1 //����5�ж�����
#define NVIC_UART5_S 0

#define NVIC_UART4_P 3 //����4�ж�����
#define NVIC_UART4_S 1

#define NVIC_UART2_P 4 //����2�ж�����
#define NVIC_UART2_S 1

#define NVIC_UART1_P 3 //����1�ж����� //gps
#define NVIC_UART1_S 0
/***********************************************/
//=========================================

#define UartSendLXIMU 	DrvUart5SendBuf
#endif

