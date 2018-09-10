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

// 存放锚节点的标号
extern uint8_t AnchorNum = 0;

/* Main function---------------------------------------*/
void main()
{
	
//	// 设置接收的阀值时间
//	TickTime_t Rx_ticktime = {RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE};
//		
    // 硬件初始化
	DeviceInit();
//	
//	// 初始化数据缓存区
//    memset( &Buffer, 0x00, BufferSize );



//    //--- 接收模式配置---
//    // r1--配置中断源
//    SX1280SetDioIrqParams(RxIrqMask, IRQ_RX_DONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
//    // r2--配置为发送模式
//    SX1280SetRx(Rx_ticktime);



////	AppState = APP_LOWPOWER;

//    while (1)
//    {

//#if defined(TX)
//		
//		Tx_Msg_2G4(PingMsg,4);

//#else
//		
//		Rx_Msg_2G4(Buffer, &BufferSize, BUFFER_SIZE);
//		
//#endif

//    }
	while(1)
	{
		
//		if(commdinfo.Commd_In_Flag == true)
//		{
//			commdinfo.Commd_In_Flag = false;
//			SendData(1,(desangle.F));
//		} 
		Round_left();
		Hal_DelayXms(1000);
		Round_right();
		Hal_DelayXms(1000);
		Round_stop();
		Hal_DelayXms(1000);
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
	
	if(getAnchorNumber(&AnchorNum) == true)
	{
		SendString(1,"The Device is a Anchor, and the number is:");
		send_16_2_str(1,AnchorNum);
		SendString(1, "\r\n---------------------\r\n" );
	}
}