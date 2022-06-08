/**
 * @file User_Com.h
 * @brief see User_Com.c
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2022-06-08
 * 
 * THINK DIFFERENTLY
 */

#ifndef _USER_COM_H
#define _USER_COM_H

#include "SysConfig.h"

void UserCom_GetOneByte(u8 data);

void UserCom_DataAnl(u8 *data_buf, u8 data_len);


#endif