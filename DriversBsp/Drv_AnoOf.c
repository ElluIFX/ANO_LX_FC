#include "Drv_AnoOf.h"

_ano_of_st ano_of;
static uint8_t _datatemp[50];
static float check_time_ms[3];
void AnoOF_Check_State(float dT_s)
{
	u8 tmp[2];
	//���Ӽ��
	if (check_time_ms[0] < 500)
	{
		check_time_ms[0]++;
		ano_of.link_sta = 1;
	}
	else
	{
		ano_of.link_sta = 0;
	}
	//���ݼ��1
	if (check_time_ms[1] < 500)
	{
		check_time_ms[1]++;
		tmp[0] = 1;
	}
	else
	{
		tmp[0] = 0;
	}
	//���ݼ��2
	if (check_time_ms[2] < 500)
	{
		check_time_ms[2]++;
		tmp[1] = 1;
	}
	else
	{
		tmp[1] = 0;
	}
	//���ù���״̬
	if (tmp[0] && tmp[1])
	{
		ano_of.work_sta = 1;
	}
	else
	{
		ano_of.work_sta = 0;
	}
}

//AnoOF_GetOneByte�ǳ������ݽ�������������ÿ���յ�һ�ֽڹ������ݣ����ñ�����һ�Σ������������Ǵ����յ�������
//����������α����ã����ս��յ�������һ֡���ݺ󣬻��Զ��������ݽ�������AnoOF_DataAnl
void AnoOF_GetOneByte(uint8_t data)
{
	static u8 _data_len = 0, _data_cnt = 0;
	static u8 rxstate = 0;

	if (rxstate == 0 && data == 0xAA)
	{
		rxstate = 1;
		_datatemp[0] = data;
	}
	else if (rxstate == 1 && (data == HW_TYPE || data == HW_ALL))
	{
		rxstate = 2;
		_datatemp[1] = data;
	}
	else if (rxstate == 2)
	{
		rxstate = 3;
		_datatemp[2] = data;
	}
	else if (rxstate == 3 && data < 250)
	{
		rxstate = 4;
		_datatemp[3] = data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if (rxstate == 4 && _data_len > 0)
	{
		_data_len--;
		_datatemp[4 + _data_cnt++] = data;
		if (_data_len == 0)
			rxstate = 5;
	}
	else if (rxstate == 5)
	{
		rxstate = 6;
		_datatemp[4 + _data_cnt++] = data;
	}
	else if (rxstate == 6)
	{
		rxstate = 0;
		_datatemp[4 + _data_cnt] = data;
		//		DT_data_cnt = _data_cnt+5;
		//
		AnoOF_DataAnl(_datatemp, _data_cnt + 5); //
	}
	else
	{
		rxstate = 0;
	}
}
//AnoOF_DataAnlΪ�������ݽ�������������ͨ���������õ�����ģ������ĸ�������
//�������ݵ����壬�������������ģ��ʹ���ֲᣬ����ϸ�Ľ���

static void AnoOF_DataAnl(uint8_t *data, uint8_t len)
{
	u8 check_sum1 = 0, check_sum2 = 0;
	if (*(data + 3) != (len - 6)) //�ж����ݳ����Ƿ���ȷ
		return;
	for (u8 i = 0; i < len - 2; i++)
	{
		check_sum1 += *(data + i);
		check_sum2 += check_sum1;
	}
	if ((check_sum1 != *(data + len - 2)) || (check_sum2 != *(data + len - 1))) //�ж�sumУ��
		return;
	//================================================================================

	if (*(data + 2) == 0X51) //������Ϣ
	{
		if (*(data + 4) == 0) //ԭʼ������Ϣ
		{
			ano_of.of0_sta = *(data + 5);
			ano_of.of0_dx = *(data + 6);
			ano_of.of0_dy = *(data + 7);
			ano_of.of_quality = *(data + 8);
		}
		else if (*(data + 4) == 1) //�߶��ںϺ������Ϣ
		{
			ano_of.of1_sta = *(data + 5);
			ano_of.of1_dx = *((s16 *)(data + 6));
			ano_of.of1_dy = *((s16 *)(data + 8));
			ano_of.of_quality = *(data + 10);
			//
			check_time_ms[1] = 0;
			ano_of.of_update_cnt++;
		}
		else if (*(data + 4) == 2) //�ߵ��ںϺ������Ϣ
		{
			ano_of.of2_sta = *(data + 5);
			ano_of.of2_dx = *((s16 *)(data + 6));
			ano_of.of2_dy = *((s16 *)(data + 8));
			ano_of.of2_dx_fix = *((s16 *)(data + 10));
			ano_of.of2_dy_fix = *((s16 *)(data + 12));
			ano_of.intergral_x = *((s16 *)(data + 14));
			ano_of.intergral_y = *((s16 *)(data + 16));
			ano_of.of_quality = *(data + 18);
			//
		}
	}
	else if (*(data + 2) == 0X34) //�߶���Ϣ
	{
		ano_of.of_alt_cm = *((u32 *)(data + 7));
		//
		check_time_ms[2] = 0;
		ano_of.alt_update_cnt++;
	}
	else if (*(data + 2) == 0X01) //��������
	{
		ano_of.acc_data_x = *((s16 *)(data + 4));
		ano_of.acc_data_y = *((s16 *)(data + 6));
		ano_of.acc_data_z = *((s16 *)(data + 8));
		ano_of.gyr_data_x = *((s16 *)(data + 10));
		ano_of.gyr_data_y = *((s16 *)(data + 12));
		ano_of.gyr_data_z = *((s16 *)(data + 14));
		//shock_sta+16
	}
	else if (*(data + 2) == 0X04) //��̬��Ϣ
	{
		//��Ԫ����ʽ
		ano_of.quaternion[0] = (*((s16 *)(data + 4))) * 0.0001f;
		ano_of.quaternion[1] = (*((s16 *)(data + 6))) * 0.0001f;
		ano_of.quaternion[2] = (*((s16 *)(data + 8))) * 0.0001f;
		ano_of.quaternion[3] = (*((s16 *)(data + 10))) * 0.0001f;
	}
}
