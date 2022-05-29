#include "ANO_DT_LX.h"
#include "ANO_LX.h"
#include "Drv_RcIn.h"
#include "LX_FC_EXT_Sensor.h"
#include "Drv_led.h"
#include "LX_FC_State.h"
#include "Drv_Uart.h"

/*==========================================================================
 * ����    �������ɿ�ͨ��������
 * ����ʱ�䣺2020-01-22 
 * ����		 �������ƴ�-�費˼
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
 * ��Ŀ������18084888982��18061373080
============================================================================
 * �����ƴ��ŶӸ�л��ҵ�֧�֣���ӭ��ҽ�Ⱥ���ཻ�������ۡ�ѧϰ��
 * �������������в��õĵط�����ӭ����ש�������
 * �������������ã�����������Ƽ���֧�����ǡ�
 * ������Դ������뻶ӭ�������á��������չ��������ϣ������ʹ��ʱ��ע��������
 * ����̹������С�˳����ݣ��������������ˮ���������ӣ�Ҳ��δ�й�Ĩ��ͬ�е���Ϊ��  
 * ��Դ���ף�����������ף�ϣ����һ������ء����ﻥ������ͬ������
 * ֻ������֧�֣������������ø��á�  
===========================================================================*/

u8 send_buffer[50]; //�������ݻ���
_dt_st dt;

//===================================================================
void ANO_DT_Init(void)
{
	//========��ʱ����
	//
	dt.fun[0x0d].D_Addr = 0xff;
	dt.fun[0x0d].fre_ms = 100;	  //�������͵�����100ms
	dt.fun[0x0d].time_cnt_ms = 1; //���ó�ʼ��λ����λ1ms
	//
	dt.fun[0x40].D_Addr = 0xff;
	dt.fun[0x40].fre_ms = 20;	  //�������͵�����100ms
	dt.fun[0x40].time_cnt_ms = 0; //���ó�ʼ��λ����λ1ms
	//========�ⲿ����
	//
	dt.fun[0x30].D_Addr = 0xff;
	dt.fun[0x30].fre_ms = 0;	  //0 ���ⲿ����
	dt.fun[0x30].time_cnt_ms = 0; //���ó�ʼ��λ����λ1ms
	//
	dt.fun[0x33].D_Addr = 0xff;
	dt.fun[0x33].fre_ms = 0;	  //0 ���ⲿ����
	dt.fun[0x33].time_cnt_ms = 0; //���ó�ʼ��λ����λ1ms
	//
	dt.fun[0x34].D_Addr = 0xff;
	dt.fun[0x34].fre_ms = 0;	  //0 ���ⲿ����
	dt.fun[0x34].time_cnt_ms = 0; //���ó�ʼ��λ����λ1ms
	//
	dt.fun[0x41].D_Addr = 0xff;
	dt.fun[0x41].fre_ms = 0;	  //0 ���ⲿ����
	dt.fun[0x41].time_cnt_ms = 0; //���ó�ʼ��λ����λ1ms
	//
	dt.fun[0xe0].D_Addr = 0xff;
	dt.fun[0xe0].fre_ms = 0;	  //0 ���ⲿ����
	dt.fun[0xe0].time_cnt_ms = 0; //���ó�ʼ��λ����λ1ms
	//
	dt.fun[0xe2].D_Addr = 0xff;
	dt.fun[0xe2].fre_ms = 0;	  //0 ���ⲿ����
	dt.fun[0xe2].time_cnt_ms = 0; //���ó�ʼ��λ����λ1ms
}

//���ݷ��ͽӿ�
static void ANO_DT_LX_Send_Data(u8 *dataToSend, u8 length)
{
	//
	UartSendLXIMU(dataToSend, length);
}

//===================================================================
//���ݽ��ճ���
//===================================================================
static u8 DT_RxBuffer[256], DT_data_cnt = 0;
void ANO_DT_LX_Data_Receive_Prepare(u8 data)
{
	static u8 _data_len = 0, _data_cnt = 0;
	static u8 rxstate = 0;

	//�ж�֡ͷ�Ƿ���������Э���0xAA
	if (rxstate == 0 && data == 0xAA)
	{
		rxstate = 1;
		DT_RxBuffer[0] = data;
	}
	//�ж��ǲ��Ƿ��͸���ģ������ݻ����ǹ㲥����
	else if (rxstate == 1 && (data == HW_TYPE || data == HW_ALL))
	{
		rxstate = 2;
		DT_RxBuffer[1] = data;
	}
	//����֡CMD�ֽ�
	else if (rxstate == 2)
	{
		rxstate = 3;
		DT_RxBuffer[2] = data;
	}
	//�������ݳ����ֽ�
	else if (rxstate == 3 && data < 250)
	{
		rxstate = 4;
		DT_RxBuffer[3] = data;
		_data_len = data;
		_data_cnt = 0;
	}
	//����������
	else if (rxstate == 4 && _data_len > 0)
	{
		_data_len--;
		DT_RxBuffer[4 + _data_cnt++] = data;
		if (_data_len == 0)
			rxstate = 5;
	}
	//����У���ֽ�1
	else if (rxstate == 5)
	{
		rxstate = 6;
		DT_RxBuffer[4 + _data_cnt++] = data;
	}
	//����У���ֽ�2����ʾһ֡���ݽ�����ϣ��������ݽ�������
	else if (rxstate == 6)
	{
		rxstate = 0;
		DT_RxBuffer[4 + _data_cnt] = data;
		DT_data_cnt = _data_cnt + 5;
		//ano_dt_data_ok = 1;
		ANO_DT_LX_Data_Receive_Anl(DT_RxBuffer, DT_data_cnt);
	}
	else
	{
		rxstate = 0;
	}
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl������Э�����ݽ������������������Ƿ���Э���ʽ��һ������֡���ú��������ȶ�Э�����ݽ���У��
//У��ͨ��������ݽ��н�����ʵ����Ӧ����
//�˺������Բ����û����е��ã��ɺ���ANO_Data_Receive_Prepare�Զ�����
static void ANO_DT_LX_Data_Receive_Anl(u8 *data, u8 len)
{
	u8 check_sum1 = 0, check_sum2 = 0;
	//�ж����ݳ����Ƿ���ȷ
	if (*(data + 3) != (len - 6))
		return;
	//�����յ������ݼ���У���ֽ�1��2
	for (u8 i = 0; i < len - 2; i++)
	{
		check_sum1 += *(data + i);
		check_sum2 += check_sum1;
	}
	//�������У���ֽں��յ���У���ֽ����Աȣ���ȫһ�´���֡���ݺϷ�����һ����������������
	if ((check_sum1 != *(data + len - 2)) || (check_sum2 != *(data + len - 1))) //�ж�sumУ��
		return;
	//�ٴ��ж�֡ͷ�Լ�Ŀ���ַ�Ƿ�Ϸ�
	if (*(data) != 0xAA || (*(data + 1) != HW_TYPE && *(data + 1) != HW_ALL))
		return;
	//=============================================================================
	//����֡��CMD��Ҳ���ǵ�3�ֽڣ����ж�Ӧ���ݵĽ���
	//PWM����
	if (*(data + 2) == 0X20)
	{
		pwm_to_esc.pwm_m1 = *((u16 *)(data + 4));
		pwm_to_esc.pwm_m2 = *((u16 *)(data + 6));
		pwm_to_esc.pwm_m3 = *((u16 *)(data + 8));
		pwm_to_esc.pwm_m4 = *((u16 *)(data + 10));
		pwm_to_esc.pwm_m5 = *((u16 *)(data + 12));
		pwm_to_esc.pwm_m6 = *((u16 *)(data + 14));
		pwm_to_esc.pwm_m7 = *((u16 *)(data + 16));
		pwm_to_esc.pwm_m8 = *((u16 *)(data + 18));
	}
	//����IMU������RGB�ƹ�����
	else if (*(data + 2) == 0X0f)
	{
		led.brightness[0] = *(data + 4);
		led.brightness[1] = *(data + 5);
		led.brightness[2] = *(data + 6);
		led.brightness[3] = *(data + 7);
	}
	//�����ɿص�ǰ������״̬
	else if (*(data + 2) == 0X06)
	{
		fc_sta.fc_mode_sta = *(data + 4);
		fc_sta.unlock_sta = *(data + 5);
		fc_sta.cmd_fun.CID = *(data + 6);
		fc_sta.cmd_fun.CMD_0 = *(data + 7);
		fc_sta.cmd_fun.CMD_1 = *(data + 8);
	}
	//�����ٶ�
	else if (*(data + 2) == 0X07)
	{
		for(u8 i=0;i<6;i++)
		{
			fc_vel.byte_data[i] = *(data + 4 + i);
		}
	}
	//��̬�ǣ���Ҫ����λ������IMU��������������ܣ�
	else if (*(data + 2) == 0X03)
	{
		for(u8 i=0;i<7;i++)
		{
			fc_att.byte_data[i] = *(data + 4 + i);
		}		
	}
	//��̬��Ԫ��
	else if (*(data + 2) == 0X03)
	{
		for(u8 i=0;i<9;i++)
		{
			fc_att_qua.byte_data[i] = *(data + 4 + i);
		}			
	}
	//����������
	else if (*(data + 2) == 0X01)
	{
		/*
		acc_x = *((s16 *)(data + 4));
		acc_y = *((s16 *)(data + 6));
		acc_z = *((s16 *)(data + 8));
		gyr_x = *((s16 *)(data + 10));
		gyr_y = *((s16 *)(data + 12));
		gyr_z = *((s16 *)(data + 14));	
		state = *(data + 16);
		*/
	}
	//����E0�����������ʽ�����ܣ��μ�����ͨ��Э��V7��
	else if (*(data + 2) == 0XE0)
	{
		//��������ID��(*(data + 4)) ����ִ�в�ͬ������
		switch (*(data + 4))
		{
		case 0x01:
		{
		}
		break;
		case 0x02:
		{
		}
		break;
		case 0x10:
		{
		}
		break;
		case 0x11:
		{
		}
		break;
		default:
			break;
		}
		//�յ��������Ҫ���ض�Ӧ��Ӧ����Ϣ��Ҳ����CK_Back����
		dt.ck_send.ID = *(data + 4);
		dt.ck_send.SC = check_sum1;
		dt.ck_send.AC = check_sum2;
		CK_Back(SWJ_ADDR, &dt.ck_send);
	}
	//�յ�����ck����
	else if (*(data + 2) == 0X00)
	{
		//�ж��յ���CK��Ϣ�ͷ��͵�CK��Ϣ�Ƿ����
		if ((dt.ck_back.ID == *(data + 4)) && (dt.ck_back.SC == *(data + 5)) && (dt.ck_back.AC == *(data + 6)))
		{
			//У��ɹ�
			dt.wait_ck = 0;
		}
	}
	//��ȡ����
	else if (*(data + 2) == 0XE1)
	{
		//��ȡ��Ҫ��ȡ�Ĳ�����id
		u16 _par = *(data + 4) + *(data + 5) * 256;
		dt.par_data.par_id = _par;
		dt.par_data.par_val = 0;
		//���͸ò���
		PAR_Back(0xff, &dt.par_data);
	}
	//д�����
	else if (*(data + 2) == 0xE2)
	{
		//Ŀǰ������ԴMCU���漰������д�룬�Ƽ����ֱ��ʹ��Դ�뷽ʽ�����Լ�����Ĳ������ʴ˴�ֻ���ض�Ӧ��CKУ����Ϣ
		//		u16 _par = *(data+4)+*(data+5)*256;
		//		u32 _val = (s32)(((*(data+6))) + ((*(data+7))<<8) + ((*(data+8))<<16) + ((*(data+9))<<24));
		//
		dt.ck_send.ID = *(data + 4);
		dt.ck_send.SC = check_sum1;
		dt.ck_send.AC = check_sum2;
		CK_Back(0xff, &dt.ck_send);
		//��ֵ����
		//Parameter_Set(_par,_val);
	}
}

//===================================================================
//���ݷ���ʵ�ֳ���
//===================================================================
static void Add_Send_Data(u8 frame_num, u8 *_cnt, u8 send_buffer[])
{
	s16 temp_data;
	s32 temp_data_32;
	//������Ҫ���͵�֡ID��Ҳ����frame_num����������ݣ���䵽send_buffer������
	switch (frame_num)
	{
	case 0x00: //CHECK����
	{
		send_buffer[(*_cnt)++] = dt.ck_send.ID;
		send_buffer[(*_cnt)++] = dt.ck_send.SC;
		send_buffer[(*_cnt)++] = dt.ck_send.AC;
	}
	break;
	case 0x0d: //�������
	{
		for (u8 i = 0; i < 4; i++)
		{
			send_buffer[(*_cnt)++] = fc_bat.byte_data[i];
		}
	}
	break;
	case 0x30: //GPS����
	{
		//
		for (u8 i = 0; i < 23; i++)
		{
			send_buffer[(*_cnt)++] = ext_sens.fc_gps.byte[i];
		}
	}
	break;
	case 0x33: //ͨ���ٶȲ�������
	{
		//
		for (u8 i = 0; i < 6; i++)
		{
			send_buffer[(*_cnt)++] = ext_sens.gen_vel.byte[i];
		}
	}
	break;
	case 0x34: //ͨ�þ����������
	{
		//
		for (u8 i = 0; i < 7; i++)
		{
			send_buffer[(*_cnt)++] = ext_sens.gen_dis.byte[i];
		}
	}
	break;
	case 0x40: //ң������֡
	{
		for (u8 i = 0; i < 20; i++)
		{
			send_buffer[(*_cnt)++] = rc_in.rc_ch.byte_data[i];
		}
	}
	break;
	case 0x41: //ʵʱ��������֡
	{
		for (u8 i = 0; i < 14; i++)
		{
			send_buffer[(*_cnt)++] = rt_tar.byte_data[i];
		}
	}
	break;
	case 0xe0: //CMD����֡
	{
		send_buffer[(*_cnt)++] = dt.cmd_send.CID;
		for (u8 i = 0; i < 10; i++)
		{
			send_buffer[(*_cnt)++] = dt.cmd_send.CMD[i];
		}
	}
	break;
	case 0xe2: //PARA����
	{
		temp_data = dt.par_data.par_id;
		send_buffer[(*_cnt)++] = BYTE0(temp_data);
		send_buffer[(*_cnt)++] = BYTE1(temp_data);
		temp_data_32 = dt.par_data.par_val;
		send_buffer[(*_cnt)++] = BYTE0(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE1(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE2(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE3(temp_data_32);
	}
	break;
	default:
		break;
	}
}

//===================================================================

static void Frame_Send(u8 frame_num, _dt_frame_st *dt_frame)
{
	u8 _cnt = 0;

	send_buffer[_cnt++] = 0xAA;
	send_buffer[_cnt++] = dt_frame->D_Addr;
	send_buffer[_cnt++] = frame_num;
	send_buffer[_cnt++] = 0;
	//==
	//add_send_data
	Add_Send_Data(frame_num, &_cnt, send_buffer);
	//==
	send_buffer[3] = _cnt - 4;
	//==
	u8 check_sum1 = 0, check_sum2 = 0;
	for (u8 i = 0; i < _cnt; i++)
	{
		check_sum1 += send_buffer[i];
		check_sum2 += check_sum1;
	}
	send_buffer[_cnt++] = check_sum1;
	send_buffer[_cnt++] = check_sum2;
	//
	if (dt.wait_ck != 0 && frame_num == 0xe0)
	{
		dt.ck_back.ID = frame_num;
		dt.ck_back.SC = check_sum1;
		dt.ck_back.AC = check_sum2;
	}
	ANO_DT_LX_Send_Data(send_buffer, _cnt);
}
//===================================================================
//
static void Check_To_Send(u8 frame_num)
{
	//
	if (dt.fun[frame_num].fre_ms)
	{
		//
		if (dt.fun[frame_num].time_cnt_ms < dt.fun[frame_num].fre_ms)
		{
			dt.fun[frame_num].time_cnt_ms++;
		}
		else
		{
			dt.fun[frame_num].time_cnt_ms = 1;
			dt.fun[frame_num].WTS = 1; //��ǵȴ�����
		}
	}
	else
	{
		//�ȴ��ⲿ����
	}
	//
	if (dt.fun[frame_num].WTS)
	{
		dt.fun[frame_num].WTS = 0;
		//ʵ�ʷ���
		Frame_Send(frame_num, &dt.fun[frame_num]);
	}
}
//===================================================================

//CMD����
void CMD_Send(u8 dest_addr, _cmd_st *cmd)
{
	dt.fun[0xe0].D_Addr = dest_addr;
	dt.fun[0xe0].WTS = 1; //���CMD�ȴ�����
	dt.wait_ck = 1;		  //��ǵȴ�У��
}
//CHECK����
void CK_Back(u8 dest_addr, _ck_st *ck)
{
	dt.fun[0x00].D_Addr = dest_addr;
	dt.fun[0x00].WTS = 1; //���CMD�ȴ�����
}
//PARA����
void PAR_Back(u8 dest_addr, _par_st *par)
{
	dt.fun[0xe2].D_Addr = dest_addr;
	dt.fun[0xe2].WTS = 1; //���CMD�ȴ�����
}

//��ָ��û���ͳɹ�����������·��ͣ����50ms��
static u8 repeat_cnt;
static inline void CK_Back_Check()
{
	static u8 time_dly;
	if (dt.wait_ck == 1)
	{
		if (time_dly < 50) //50ms
		{
			time_dly++;
		}
		else
		{
			time_dly = 0;
			repeat_cnt++;
			if (repeat_cnt < 5)
			{
				dt.fun[0xe0].WTS = 1; //��ǵȴ����ͣ��ط�
			}
			else
			{
				repeat_cnt = 0;
				dt.wait_ck = 0;
			}
		}
	}
	else
	{
		time_dly = 0;
		repeat_cnt = 0;
	}
}

//1ms����һ�Σ�����ͨ�Ž�������
void ANO_LX_Data_Exchange_Task(float dT_s)
{
	//=====���CMD�Ƿ񷵻���У��
	CK_Back_Check();
	//=====����Ƿ񴥷�����
	Check_To_Send(0x30);
	Check_To_Send(0x33);
	Check_To_Send(0x34);
	Check_To_Send(0x40);
	Check_To_Send(0x41);
	Check_To_Send(0xe0);
	Check_To_Send(0xe2);
	Check_To_Send(0x0d);
}

//===================================================================
