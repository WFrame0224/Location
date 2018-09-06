#ifndef __MAIN_H__
#define __MAIN_H__

#include "hal.h"
#include "datatype.h"

//**********************************************************************
/* Defines---------------------------------------------*/
/*!
 * \brief Defines the buffer size, i.e. the payload size
 */
#define BUFFER_SIZE                                 100

//***************************功能函数*******************************
/*!
 * \breif 进行硬件设备的初始化，串口、定时器、2.4G射频初始化
 */
void DeviceInit();
#endif