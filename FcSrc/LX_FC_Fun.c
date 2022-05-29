#include "LX_FC_Fun.h"
#include "LX_FC_State.h"
#include "ANO_DT_LX.h"

/*==========================================================================
 * ����    �������ɿػ�������������
 * ����ʱ�䣺2020-03-31 
 * ����		 �������ƴ�-Jyoun
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

//////////////////////////////////////////////////////////////////////
//�������ܺ���
//////////////////////////////////////////////////////////////////////
//
u8 FC_Unlock()
{
	//
	fc_sta.unlock_cmd = 1; //����
	//��Э�鷢��ָ��
	if (dt.wait_ck == 0) //û�������ȴ�У���CMDʱ�ŷ��ͱ�CMD
	{
		dt.cmd_send.CID = 0x10;
		dt.cmd_send.CMD[0] = 0x00;
		dt.cmd_send.CMD[1] = 0x01;
		CMD_Send(0XFF, &dt.cmd_send);
		return 1;
	}
	else
	{
		return 0;
	}
}
//
u8 FC_Lock()
{
	//
	fc_sta.unlock_cmd = 0; //����
	//��Э�鷢��ָ��
	if (dt.wait_ck == 0) //û�������ȴ�У���CMDʱ�ŷ��ͱ�CMD
	{
		dt.cmd_send.CID = 0x10;
		dt.cmd_send.CMD[0] = 0x00;
		dt.cmd_send.CMD[1] = 0x02;
		CMD_Send(0XFF, &dt.cmd_send);
		return 1;
	}
	else
	{
		return 0;
	}
}

//�ı�ɿ�ģʽ(ģʽ0-3)
u8 LX_Change_Mode(u8 new_mode)
{
	static u8 old_mode;
	if (old_mode != new_mode)
	{
		//
		if (dt.wait_ck == 0) //û�������ȴ�У���CMDʱ�ŷ��ͱ�CMD
		{
			old_mode = fc_sta.fc_mode_cmd = new_mode;
			//��Э�鷢��ָ��
			dt.cmd_send.CID = 0X01;
			dt.cmd_send.CMD[0] = 0X01;
			dt.cmd_send.CMD[1] = 0X01;
			dt.cmd_send.CMD[2] = fc_sta.fc_mode_cmd;
			CMD_Send(0xff, &dt.cmd_send);
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else //�Ѿ��ڵ�ǰģʽ
	{
		return 1;
	}
}

//һ������
//��Ҫע�⣬�̿�ģʽ�²���ִ�з���
u8 OneKey_Return_Home()
{
	//
	if (dt.wait_ck == 0) //û�������ȴ�У���CMDʱ�ŷ��ͱ�CMD
	{
		//��Э�鷢��ָ��
		dt.cmd_send.CID = 0X10;
		dt.cmd_send.CMD[0] = 0X00;
		dt.cmd_send.CMD[1] = 0X07;
		CMD_Send(0xff, &dt.cmd_send);
		return 1;
	}
	else
	{
		return 0;
	}
}

//һ�����(�߶�cm)
u8 OneKey_Takeoff(u16 height_cm)
{
	//
	if (dt.wait_ck == 0) //û�������ȴ�У���CMDʱ�ŷ��ͱ�CMD
	{
		//��Э�鷢��ָ��
		dt.cmd_send.CID = 0X10;
		dt.cmd_send.CMD[0] = 0X00;
		dt.cmd_send.CMD[1] = 0X05;
		dt.cmd_send.CMD[2] = BYTE0(height_cm);
		dt.cmd_send.CMD[3] = BYTE1(height_cm);
		CMD_Send(0xff, &dt.cmd_send);
		return 1;
	}
	else
	{
		return 0;
	}
}
//һ������
u8 OneKey_Land()
{
	//
	if (dt.wait_ck == 0) //û�������ȴ�У���CMDʱ�ŷ��ͱ�CMD
	{
		//��Э�鷢��ָ��
		dt.cmd_send.CID = 0X10;
		dt.cmd_send.CMD[0] = 0X00;
		dt.cmd_send.CMD[1] = 0X06;
		CMD_Send(0xff, &dt.cmd_send);
		return 1;
	}
	else
	{
		return 0;
	}
}
//ƽ��(����cm���ٶ�cmps������Ƕ�0-360��)
u8 Horizontal_Move(u16 distance_cm, u16 velocity_cmps, u16 dir_angle_0_360)
{
	//
	if (dt.wait_ck == 0) //û�������ȴ�У���CMDʱ�ŷ��ͱ�CMD
	{
		//��Э�鷢��ָ��
		dt.cmd_send.CID = 0X10;
		dt.cmd_send.CMD[0] = 0X02;
		dt.cmd_send.CMD[1] = 0X03;
		//
		dt.cmd_send.CMD[2] = BYTE0(distance_cm);
		dt.cmd_send.CMD[3] = BYTE1(distance_cm);
		dt.cmd_send.CMD[4] = BYTE0(velocity_cmps);
		dt.cmd_send.CMD[5] = BYTE1(velocity_cmps);
		dt.cmd_send.CMD[6] = BYTE0(dir_angle_0_360);
		dt.cmd_send.CMD[7] = BYTE1(dir_angle_0_360);
		//
		CMD_Send(0xff, &dt.cmd_send);
		return 1;
	}
	else
	{
		return 0;
	}
}

//ˮƽУ׼
u8 Horizontal_Calibrate()
{
	//
	if (dt.wait_ck == 0) //û�������ȴ�У���CMDʱ�ŷ��ͱ�CMD
	{
		//��Э�鷢��ָ��
		dt.cmd_send.CID = 0X01;
		dt.cmd_send.CMD[0] = 0X00;
		dt.cmd_send.CMD[1] = 0X03;
		CMD_Send(0xff, &dt.cmd_send);
		return 1;
	}
	else
	{
		return 0;
	}
}

//������У׼
u8 Mag_Calibrate()
{
	//
	if (dt.wait_ck == 0) //û�������ȴ�У���CMDʱ�ŷ��ͱ�CMD
	{
		//��Э�鷢��ָ��
		dt.cmd_send.CID = 0X01;
		dt.cmd_send.CMD[0] = 0X00;
		dt.cmd_send.CMD[1] = 0X04;
		CMD_Send(0xff, &dt.cmd_send);
		return 1;
	}
	else
	{
		return 0;
	}
}

//6����ٶ�У׼
u8 ACC_Calibrate()
{
	//
	if (dt.wait_ck == 0) //û�������ȴ�У���CMDʱ�ŷ��ͱ�CMD
	{
		//��Э�鷢��ָ��
		dt.cmd_send.CID = 0X01;
		dt.cmd_send.CMD[0] = 0X00;
		dt.cmd_send.CMD[1] = 0X05;
		CMD_Send(0xff, &dt.cmd_send);
		return 1;
	}
	else
	{
		return 0;
	}
}

//������У׼
u8 GYR_Calibrate()
{
	//
	if (dt.wait_ck == 0) //û�������ȴ�У���CMDʱ�ŷ��ͱ�CMD
	{
		//��Э�鷢��ָ��
		dt.cmd_send.CID = 0X01;
		dt.cmd_send.CMD[0] = 0X00;
		dt.cmd_send.CMD[1] = 0X02;
		CMD_Send(0xff, &dt.cmd_send);
		return 1;
	}
	else
	{
		return 0;
	}
}
