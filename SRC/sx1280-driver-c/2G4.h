#ifndef __2G4_H__
#define __2G4_H__

#include "datatype.h"
//**********************************************************************
/* Defines---------------------------------------------*/
#define RF_BL_ADV_CHANNEL_38                        2400000000 // Hz

/*!
 * \brief Defines the nominal frequency
 */
#define RF_FREQUENCY                                RF_BL_ADV_CHANNEL_38 // Hz

/*!
 * \brief Defines the output power in dBm
 *
 * \remark The range of the output power is [-18..+13] dBm
 */
#define TX_OUTPUT_POWER                             13

/*!
 * \brief Number of tick size steps for tx timeout
 */
#define TX_TIMEOUT_VALUE                            10000 // ms

/*!
 * \brief Number of tick size steps for rx timeout
 */
#define RX_TIMEOUT_VALUE                            0xffff // ms

/*!
 * \brief Size of ticks (used for Tx and Rx timeout)
 */
#define RX_TIMEOUT_TICK_SIZE                        RADIO_TICK_SIZE_1000_US

/*!
 * \brief Defines the size of the token defining message type in the payload
 */
#define PINGPONGSIZE                                4

/*!
 * \brief Defines the states of the application
 */
typedef enum
{
    APP_LOWPOWER,
    APP_RX,
    APP_RX_TIMEOUT,
    APP_RX_ERROR,
    APP_TX,
    APP_TX_TIMEOUT,
}AppStates_t;

//***************************功能函数*******************************
/*!
 * \brief 2.4G射频通信模块的初始化配置，同时进行Lora通信参数的设置
 */
void Init_2G4();

/*!
 * \brief 进行数据的发送
 */
void Tx_Msg_2G4(uint8_t* msg, uint8_t msg_size);

/*!
 * \brief 进行数据的接收
 */
int8_t Rx_Msg_2G4(uint8_t* Buffer, uint8_t* BufferSize, uint8_t maxmsize );




/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( void );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError();

#endif