#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"

void UserTask_OneKeyCmd(void)
{
    //////////////////////////////////////////////////////////////////////
    //һ�����/��������
    //////////////////////////////////////////////////////////////////////
    //�þ�̬������¼һ�����/����ָ���Ѿ�ִ�С�
    static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;
    static u8 mission_step;
    //�ж���ң���źŲ�ִ��
    if (rc_in.fail_safe == 0)
    {
        //�жϵ�6ͨ������λ�� 1300<CH_6<1700
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700)
        {
            //��û��ִ��
            if (one_key_takeoff_f == 0)
            {
                //����Ѿ�ִ��
                one_key_takeoff_f =
                    //ִ��һ�����
                    OneKey_Takeoff(100); //������λ�����ף� 0��Ĭ����λ�����õĸ߶ȡ�
            }
        }
        else
        {
            //��λ��ǣ��Ա��ٴ�ִ��
            one_key_takeoff_f = 0;
        }
        //
        //�жϵ�6ͨ������λ�� 800<CH_6<1200
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            //��û��ִ��
            if (one_key_land_f == 0)
            {
                //����Ѿ�ִ��
                one_key_land_f =
                    //ִ��һ������
                    OneKey_Land();
            }
        }
        else
        {
            //��λ��ǣ��Ա��ٴ�ִ��
            one_key_land_f = 0;
        }
        //�жϵ�6ͨ������λ�� 1700<CH_6<2000
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        {
            //��û��ִ��
            if (one_key_mission_f == 0)
            {
                //����Ѿ�ִ��
                one_key_mission_f = 1;
                //��ʼ����
                mission_step = 1;
            }
        }
        else
        {
            //��λ��ǣ��Ա��ٴ�ִ��
            one_key_mission_f = 0;
        }
        //
        if (one_key_mission_f == 1)
        {
        }
        else
        {
            mission_step = 0;
        }
    }
    ////////////////////////////////////////////////////////////////////////
}
