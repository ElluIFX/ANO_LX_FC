/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * ����    �������ƴ�
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
 * ����    ����ʱ������
**********************************************************************************/

#include "Drv_Timer.h"
#include "Drv_led.h"

#define SYS_TIMx_IRQn TIM7_IRQn
#define SYS_TIMx TIM7
#define SYS_RCC_TIMx RCC_APB1Periph_TIM7

void TIM_CONF(u16 period_ms) //APB1  84M
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    /* ʹ��ʱ�� */
    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx, ENABLE);

    TIM_DeInit(SYS_TIMx);

    /* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */
    TIM_TimeBaseStructure.TIM_Period = period_ms;

    /* �ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж� */
    /* ʱ��Ԥ��Ƶ��Ϊ */
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;

    /* ���ⲿʱ�ӽ��в�����ʱ�ӷ�Ƶ,����û���õ� */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���

    TIM_TimeBaseInit(SYS_TIMx, &TIM_TimeBaseStructure);

    TIM_ClearFlag(SYS_TIMx, TIM_FLAG_Update);

    TIM_ITConfig(SYS_TIMx, TIM_IT_Update, ENABLE);

    TIM_Cmd(SYS_TIMx, ENABLE);

    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx, DISABLE); /*�ȹرյȴ�ʹ��*/
}
void TIM_NVIC()
{
    NVIC_InitTypeDef NVIC_InitStructure;

    //    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel = SYS_TIMx_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_TIME_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_TIME_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void DrvTimerFcInit(void)
{
    TIM_CONF(1000);
    TIM_NVIC();

    /* TIM7 ���¿�ʱ�ӣ���ʼ��ʱ */
    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx, ENABLE);
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
