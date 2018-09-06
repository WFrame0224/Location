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

/* defines---------------------------------------------*/

//#define TX
#define RX

/* Variables-------------------------------------------*/
extern uint TimeIndex;
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
uint8_t idata Buffer[BUFFER_SIZE];
/*!
 * \brief Define the possible message type for this application
 */
const uint8_t idata PingMsg[] = "PING";
const uint8_t idata PongMsg[] = "PONG";

int8_t RSSI[10] = {0x00};

/* Main function---------------------------------------*/
void main()
{
	
	// 设置接收的阀值时间
	TickTime_t Rx_ticktime = {RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE};
		
    // 硬件初始化
	DeviceInit();
	
	// 初始化数据缓存区
    memset( &Buffer, 0x00, BufferSize );

#if defined(TX)

    //---发送模式配置----
    // t1--配置发送参数
    SX1280SetTxParams(TX_OUTPUT_POWER, RADIO_RAMP_02_US);

#else

    //--- 接收模式配置---
    // r1--配置中断源
    SX1280SetDioIrqParams(RxIrqMask, IRQ_RX_DONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
    // r2--配置为发送模式
    SX1280SetRx(Rx_ticktime);

#endif

//	AppState = APP_LOWPOWER;

    while (1)
    {

#if defined(TX)
		
		Tx_Msg_2G4(PingMsg,4);

#else
		
		Rx_Msg_2G4(Buffer, &BufferSize, BUFFER_SIZE);
		
#endif

    }

}

void DeviceInit()
{
	// 硬件初始化
	
	// 串口初始化
    Init_Uart();
	SendString(1, "-----------------------Initing-------------------\r\n" );
	SendString(1, "Uart initialization completed...........\r\n" );
	// 定时器初始化
    InitTimer0();
	SendString(1, "Timer initialization completed...........\r\n" );
	// 2.4G射频初始化
    Init_2G4();
	SendString(1, "2.4G initialization completed...........\r\n" );
	
    SendString(1, "---------------------Init finished----------------\r\n" );
}