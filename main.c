/**
 * File Name:main.c
 * Description:本程为定位节点的主程序
 *
 * @Author:	王非
 * @Date:	2018.08.12
 * ***************************************
 * Function:程序功能.....
 *
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


/* Variables-------------------------------------------*/
/*!
 * \brief The size of the buffer
 */
uint8_t BufferSize = BUFFER_SIZE;

/*!
 * \brief Mask of IRQs to listen to in rx mode
 */
extern uint16_t idata RxIrqMask;
/*!
 * \brief Mask of IRQs to listen to in tx mode
 */
extern uint16_t idata TxIrqMask;

/*!
 * \brief The buffer
 */
uint8_t Buffer[BUFFER_SIZE];
/*!
 * \brief Define the possible message type for this application
 */
const uint8_t idata PingMsg[] = "PING";
const uint8_t idata PongMsg[] = "PONG";


// 存放中心站由433M信道传来的启动帧和电机控制帧
extern CommdInfo commdinfo;

// 存放需求角度以及角度旋转的方向
extern DesAngle desangle;


/* Main function---------------------------------------*/
void main()
{
    int16_t angel1 = 0;
    uint8_t angle0[2] = {0x00};
	DesAngle initangel = {'+',72};
    // 硬件初始化
    DeviceInit();

    angel1 = getCurrentAngle();
    // 高位
    angle0[0] = (angel1 >> 8) & 0xff;
	// 地位
    angle0[1] = angel1 & 0xff;
    SendString(1, "The current angle:");
    SendHex2Ascills(1, angle0, 2);
    SendString(1, "\r\n");
    Hal_DelayXms(1000);
	
	InitRound(initangel);
	
	angel1 = getCurrentAngle();
    // 高位
    angle0[0] = (angel1 >> 8) & 0xff;
	// 地位
    angle0[1] = angel1 & 0xff;
    SendString(1, "The current angle:");
    SendHex2Ascills(1, angle0, 2);
    SendString(1, "\r\n");
    Hal_DelayXms(1000);

    while(1)
    {
        ;
    }
}

/* Related functions---------------------------------------*/

void DeviceInit()
{
    // 硬件初始化

    // 串口初始化
    Init_Uart();
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