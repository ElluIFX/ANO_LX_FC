/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * ����    �������ƴ�
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
 * ����    ��SPI����
**********************************************************************************/
#include "Drv_adc.h"

uint16_t AdcValue[50] = {0};
static void initAdcDMA(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	//
	DMA_StructInit(&DMA_InitStructure);
	//
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); /*DMA2��ʱ��ʹ��*/
	while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE)
		; /*�ȴ�DMA��������*/

	DMA_InitStructure.DMA_Channel = DMA_Channel_0;								/*DMAͨ��0*/
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_BASE + 0x4C;		/*�����ַ*/
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&AdcValue;				/*��ȡ����ַ*/
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;						/*��������赽�ڴ�*/
	DMA_InitStructure.DMA_BufferSize = 50;										/*���ݴ��������Ϊ50*/
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			/*��ַ������*/
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						//DMA_MemoryInc_Disable;/*��ַ������*/
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; /*���ݳ��Ȱ���*/
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;			/*���ݳ��Ȱ���*/
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;							/*�����ȼ�*/
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;								/*ѭ��ģʽ*/
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;						/*��ֹFIFO*/
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;			/*FIFO��ֵ*/
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;					/*���δ���*/
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;			/*���δ���*/
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);									/**/
	DMA_Cmd(DMA2_Stream0, ENABLE);												//����DMA����
}

void DrvAdcInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	//
	ADC_StructInit(&ADC_InitStructure);
	//
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  //ʹ��ADCʱ��
	initAdcDMA();
	/*��ʼ��ADC1ͨ��3 ��IO��*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	 /*ģ������*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;		 /*ͨ��5*/
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; /*����������*/
	GPIO_Init(GPIOC, &GPIO_InitStructure);			 /*��ʼ��*/
	/*ͨ�ÿ��ƼĴ���������*/
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;		 /*DMAʧ��*/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;					 /*����ģʽ*/
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;					 /*APB2��4��Ƶ ��84/4=21M*/
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; /*���������׶ε���ʱ5��ʱ��*/
	ADC_CommonInit(&ADC_CommonInitStructure);									 /*��ʼ��*/
	/*��ʼ��ADC1*/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;						/*12λģʽ*/
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;								/*ɨ��ģʽ*/
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;							/*����ת��*/
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; /*��ֹ������� ʹ���������*/
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;						/*�Ҷ���*/
	ADC_InitStructure.ADC_NbrOfConversion = 1;									/*2ͨ�� 1*/
	ADC_Init(ADC1, &ADC_InitStructure);											/*��ʼ��*/

	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_480Cycles); /*���ù���ͨ��3 �������� ����ʱ�� */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);							 //Դ���ݱ仯ʱ����DMA����
	ADC_Cmd(ADC1, ENABLE);														 /*����ת��*/
	ADC_DMACmd(ADC1, ENABLE);													 //ʹ��ADC����
	ADC_SoftwareStartConv(ADC1);												 /*�������ת��*/
}

float Drv_AdcGetBatVot(void)
{
#define UP_R 10 //10K
#define DW_R 1	//1K
	//
	float tmp = 0;
	for (u8 i = 0; i < 50; i++)
	{
		tmp += AdcValue[i] * 0.02f;
	}
	//
	return tmp / 4096 * 3300 * (UP_R + DW_R) / DW_R * 0.001f; 
}
