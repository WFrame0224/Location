/**
 * Descripction：本程序实现了Lora模式的广播通信，锚节点只是用来发送数据，点位节点用来接收数据，保存RSSI
 *  属于最上层的文件，依赖于radio和sx1280相关文件，接收节点和发送节点依靠宏定义来进行区分
 * FileName:2G4.c 
 * Author：王非
 * Date：2018.09.04
 */

#include <string.h>
#include <intrins.h>
#include "2G4.h"
#include "radio.h"
#include "sx1280.h"
#include "function.h"
#include "hal.h"
#include "uart.h"

/* defines---------------------------------------------*/

#define Anchor

/* Variables-------------------------------------------*/
extern uint TimeIndex;


/*!
 * \brief All the callbacks are stored in a structure,重新写的回调函数
 */
RadioCallbacks_t Callbacks =
{
        &OnTxDone,    // txDone
        &OnRxDone,    // rxDone
        0x00,         // syncWordDone
        0x00,         // headerDone
        &OnTxTimeout, // txTimeout
        &OnRxTimeout, // rxTimeout
        &OnRxError,   // rxError
        0x00,         // rangingDone
        0x00,         // cadDone
};

/*!
 * \brief Mask of IRQs to listen to in rx mode
 */
extern uint16_t idata RxIrqMask = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;

/*!
 * \brief Mask of IRQs to listen to in tx mode
 */
extern uint16_t idata TxIrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;

/*!
 * \brief The State of the application
 */
AppStates_t idata AppState = APP_LOWPOWER;

PacketStatus_t idata packetStatus;
uint16_t irqstatus = 0;

/* functions----------------------------------- */

void Init_2G4()
{
    // 定义包参数
    PacketParams_t idata packetParams;
    // 定义调制方式
    ModulationParams_t idata modulationParams;
    

    // let DC/DC power ramp up,此处最好采用定时器硬件定时
    Delay_Xms(500);

    // 射频初始化
    SX1280Init(&Callbacks);
    SX1280SetRegulatorMode(USE_DCDC);

    // 采用Lora模式----------------------------------------------
    // 初始化LORA相关参数
    modulationParams.PacketType = PACKET_TYPE_LORA;
    modulationParams.Params.LoRa.SpreadingFactor = LORA_SF9;
    modulationParams.Params.LoRa.Bandwidth = LORA_BW_1600;
    modulationParams.Params.LoRa.CodingRate = LORA_CR_LI_4_7;

    packetParams.PacketType = PACKET_TYPE_LORA;
    packetParams.Params.LoRa.PreambleLength = 0x12;
    packetParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
    packetParams.Params.LoRa.PayloadLength = 4;
    packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
    packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;

    // 射频模式配置---------------------------------------------
    /* 基础配置 */
    // 1--设置为待机模式
    SX1280SetStandby(STDBY_RC);
    // 2--设置包类型
    SX1280SetPacketType(modulationParams.PacketType);
    // 3--设置频率参数
    SX1280SetRfFrequency(RF_FREQUENCY);
    // 4--设置数据存储区基址
    SX1280SetBufferBaseAddresses(0x00, 0x00);
    // 5--设置调制参数
    SX1280SetModulationParams(&modulationParams);
    // 6--设置数据包类型
    SX1280SetPacketParams(&packetParams);
	
#if defined(Anchor)

    //---发送模式配置----
    // t1--配置发送参数
    SX1280SetTxParams(TX_OUTPUT_POWER, RADIO_RAMP_02_US);

#endif
}
void Tx_Msg_2G4(uint8_t *msg, uint8_t msg_size)
{
    // 设置接收和发送的阀值时间
	TickTime_t Tx_ticktime = {RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE};

    // t2--配置发送中断源
    SX1280SetDioIrqParams(TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
    // t3，4--填充要发送的数据，并设置为发送模式
    SX1280SendPayload(msg, msg_size, Tx_ticktime);

    SendString(1, msg);
    SendString(1, "\r\n");
    Delay_Xms(1000);
}
int8_t Rx_Msg_2G4(uint8_t* Buffer, uint8_t* BufferSize, uint8_t maxmsize )
{
    uint16_t irqstatus = 0;
    // 芯片的状态标志
//    RadioStatus_t radioStatus;
	
	// 设置接收的阀值时间
	TickTime_t Rx_ticktime = {RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE};

    uint8_t status[5] = {0x00};

    int8_t rssi = 0;

    // 读取Irq状态
    irqstatus = SX1280GetIrqStatus();

    // SendString(1, "status is :");
    // send_16_2_str(1, (irqstatus >> 8));
    // send_16_2_str(1, irqstatus);
    // SendString(1, "\r\n");

    // if ((irqstatus & 0x4000) == 0x4000)
    // {
    //         irqstatus = 0;
    //     // r2--配置为接收模式
    //     SX1280SetRx(Rx_ticktime);
    // }

    // while((irqstatus & 0x0002) == 0x0002)
    // {

    //     irqstatus = 0;
    while (Radio_Dio1 == 1)
    {
		SX1280ClearIrqStatus(IRQ_RX_DONE);
		
//        // 获取状态
//        radioStatus = SX1280GetStatus();
//        SendString(1, "status is :");
//        send_16_2_str(1, radioStatus.Value);
//        SendString(1, "\r\n");
//        SX1280HalReadCommand(RADIO_GET_PACKETSTATUS, status, 5);
//        SendHex2Ascills(1, status, 5);
//        SendString(1, "\r\n");
		
		// r3--得到有效载荷 
        SX1280GetPayload(Buffer, BufferSize, maxmsize);

//        SendString(1, "BufferSize is: ");
//        send_16_2_str(1, *BufferSize);
//        SendString(1, "\r\n");

        SendString(1, "Received msg is: ");
        SendString(1, Buffer);
        SendString(1, "\r\n");

        // 初始化数据缓存区
        memset(&Buffer, 0x00, *BufferSize);

        rssi = SX1280GetRssiInst();
		
        SendString(1, "rssi is: ");
        send_16_2_str(1, rssi);
        SendString(1, "\r\n");
        SendString(1, "---------------------\r\n");
		
        // r2--配置为接收模式
        SX1280SetRx(Rx_ticktime);
    }
	return rssi;
}

void OnTxDone(void)
{
    AppState = APP_TX;
}

void OnRxDone(void)
{
    AppState = APP_RX;
}

void OnTxTimeout(void)
{
    AppState = APP_TX_TIMEOUT;
    SendString(1, "<>>>>>>>>TXE\n\r");
}

void OnRxTimeout(void)
{
    AppState = APP_RX_TIMEOUT;
}

void OnRxError()
{
    AppState = APP_RX_ERROR;
    SendString(1, "RXE<>>>>>>>>\n\r");
}
