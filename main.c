/**
 * File Name:main.c
 * Description:本程为定位节点的主程序
 *
 * @Author:	王非
 * @Date:	2018.08.12
 * ***************************************
 * Function:作为锚节点的程序，等待启动帧和控制帧命令，控制电机旋转，同时利用2.4G信道发送消息
*/

/* Inlcudes--------------------------------------------*/
#include <string.h>
#include "main.h"
#include "function.h"
#include "timer.h"
#include "uart.h"
#include "radio.h"
#include "sx1280.h"
#include "2G4.h"
#include "Anchor.h"

/* defines---------------------------------------------*/

// 定义一个版本号
const uint8_t SystemVersion[] = "V3_1636";

/* Variables-------------------------------------------*/
extern uint16_t CurrentAngle = 0x0000;

/* Main function---------------------------------------*/
void main()
{	
#ifdef UART_1
	uint8_t angle[2] = {0x00};
#endif

    // 硬件初始化
    DeviceInit();


#ifdef UART_1
	angle[0] = (CurrentAngle >> 8) & 0xff;
    angle[1] = CurrentAngle;
    SendString(1,"The CurrentAngle is:");
    SendHex2Ascills(1,angle,2);
#endif

    while(1)
    {
        Anchor_run();
    }
}

/* Related functions---------------------------------------*/

void DeviceInit()
{
    // 硬件初始化
	M0 = 0;
	M1 = 0;
    // 串口初始化
    Init_Uart();
	
	SendString(1,"\r\nThe System Version:_._.");
	SendString(1,SystemVersion);
	SendString(1,"._._\r\n\r\n");
	
    SendString(1, "--------------Initing----------\r\n" );
    SendString(1, "Uart initialization completed...........\r\n" );
    // 定时器初始化
    InitTimer0();
    SendString(1, "Timer initialization completed...........\r\n" );
    // 2.4G射频初始化
    Init_2G4();
    SendString(1, "2.4G initialization completed...........\r\n" );
    SendString(1, "-------------Init finished--------\r\n" );
	

	
    if(ReadAnchorNumber() == true)
    {
        SendString(1, "The Device is a Anchor, and the number is:");
        send_16_2_str(1, GetAnchorNumber());
        SendString(1, "\r\n---------------------\r\n" );
    }
}