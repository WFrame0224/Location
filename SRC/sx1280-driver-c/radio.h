/**
 * @Description：本程序仅定义与SX1280底层相关的函数，以及控制命令
 * @FileName：radio.h
 * @Author：王非
 * @Date:2018.08.13
 */ 

#ifndef __RADIO_H__
#define __RADIO_H__

/* includes---------------------------------- */
#include "datatype.h"
#include "hal.h"

/* pin definction */
sbit Radio_Busy = P7 ^ 3;
sbit Radio_Rest = P7 ^ 2;
sbit Radio_Dio3 = P2 ^ 3;
sbit Radio_Dio2 = P2 ^ 2;
sbit Radio_Dio1 = P2 ^ 1;

/* Hal definction */

/*!
 * \brief The address of the instruction RAM and its size
 */
#define IRAM_START_ADDRESS                          0x8000
#define IRAM_SIZE                                   0x4000

/* datatype define */
/*!
 * \brief Represents all possible opcode understood by the radio
 */
typedef enum RadioCommands_u          
{
    RADIO_GET_STATUS                        = 0xC0,
    RADIO_WRITE_REGISTER                    = 0x18,
    RADIO_READ_REGISTER                     = 0x19,
    RADIO_WRITE_BUFFER                      = 0x1A,
    RADIO_READ_BUFFER                       = 0x1B,
    RADIO_SET_SLEEP                         = 0x84,
    RADIO_SET_STANDBY                       = 0x80,
    RADIO_SET_FS                            = 0xC1,
    RADIO_SET_TX                            = 0x83,
    RADIO_SET_RX                            = 0x82,
    RADIO_SET_RXDUTYCYCLE                   = 0x94,
    RADIO_SET_CAD                           = 0xC5,
    RADIO_SET_TXCONTINUOUSWAVE              = 0xD1,
    RADIO_SET_TXCONTINUOUSPREAMBLE          = 0xD2,
    RADIO_SET_PACKETTYPE                    = 0x8A,
    RADIO_GET_PACKETTYPE                    = 0x03,
    RADIO_SET_RFFREQUENCY                   = 0x86,
    RADIO_SET_TXPARAMS                      = 0x8E,
    RADIO_SET_CADPARAMS                     = 0x88,
    RADIO_SET_BUFFERBASEADDRESS             = 0x8F,
    RADIO_SET_MODULATIONPARAMS              = 0x8B,
    RADIO_SET_PACKETPARAMS                  = 0x8C,
    RADIO_GET_RXBUFFERSTATUS                = 0x17,
    RADIO_GET_PACKETSTATUS                  = 0x1D,
    RADIO_GET_RSSIINST                      = 0x1F,
    RADIO_SET_DIOIRQPARAMS                  = 0x8D,
    RADIO_GET_IRQSTATUS                     = 0x15,
    RADIO_CLR_IRQSTATUS                     = 0x97,
    RADIO_CALIBRATE                         = 0x89,
    RADIO_SET_REGULATORMODE                 = 0x96,
    RADIO_SET_SAVECONTEXT                   = 0xD5,
    RADIO_SET_AUTOTX                        = 0x98,
    RADIO_SET_AUTOFS                        = 0x9E,
    RADIO_SET_LONGPREAMBLE                  = 0x9B,
    RADIO_SET_UARTSPEED                     = 0x9D,
    RADIO_SET_RANGING_ROLE                  = 0xA3,
}RadioCommands_t;





/* Function definction */
void SX1280HalWaitOnBusy(void);
void SX1280HalInit();
void SX1280HalIoIrqInit();
/*!
 * \brief Soft resets the radio 软件复位射频
 */
void SX1280HalReset(void);
/*!
 * \brief Clears the instruction ram memory block 清楚指令内存块
 */
void SX1280HalClearInstructionRam(void);
/*!
 * \brief Wakes up the radio 唤醒激活射频
 */
void SX120HalWakeup(void);
/*!
 * \brief Send a command that write data to the radio 给射频发送写命令
 *
 * \param [in]  command       Opcode of the command
 * \param [in]  buffer        Buffer to be send to the radio
 * \param [in]  size          Size of the buffer to send
 */
void SX1280HalWriteCommand(RadioCommands_t command, uint8_t *buffer, uint16_t buf_size);
/*!
 * \brief Send a command that read data from the radio 给射频发送读数据命令
 *
 * \param [in]  command       Opcode of the command
 * \param [out] buffer        Buffer holding data from the radio
 * \param [in]  size          Size of the buffer
 */
void SX1280HalReadCommand(RadioCommands_t command, uint8_t *buffer, uint16_t xsize);
/*!
 * \brief Write data to the radio memory 写数据到射频内存中
 *
 * \param [in]  address       The address of the first byte to write in the radio
 * \param [in]  buffer        The data to be written in radio's memory
 * \param [in]  size          The number of bytes to write in radio's memory
 */
void SX1280HalWriteRegisters(uint16_t address, uint8_t *buffer, uint16_t xsize);
/*!
 * \brief Write a single byte of data to the radio memory 单字节射频内存写入
 *
 * \param [in]  address       The address of the first byte to write in the radio
 * \param [in]  value         The data to be written in radio's memory
 */
void SX1280HalWriteRegister(uint16_t address, uint8_t value);
/*!
 * \brief Read data from the radio memory 从射频内存中读取数据
 *
 * \param [in]  address       The address of the first byte to read from the radio
 * \param [out] buffer        The buffer that holds data read from radio
 * \param [in]  size          The number of bytes to read from radio's memory
 */
void SX1280HalReadRegisters(uint16_t address, uint8_t *buffer, uint16_t xsize);
/*!
 * \brief Read a single byte of data from the radio memory 从内存中读取单字节数据
 *
 * \param [in]  address       The address of the first byte to write in the
     *                            radio
 *
 * \retval      value         The value of the byte at the given address in
     *                            radio's memory
 */
uint8_t SX1280HalReadRegister(uint16_t address);
/*!
 * \brief Write data to the buffer holding the payload in the radio
 *  将数据写入保持在无线电中的有效负载的缓冲器中
 * \param [in]  offset        The offset to start writing the payload
 * \param [in]  buffer        The data to be written (the payload)
 * \param [in]  size          The number of byte to be written
 */
void SX1280HalWriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t xsize);
/*!
 * \brief Read data from the buffer holding the payload in the radio
 *  从保持无线电中的有效载荷的缓冲区中读取数据
 * \param [in]  offset        The offset to start reading the payload
 * \param [out] buffer        A pointer to a buffer holding the data from the radio
 * \param [in]  size          The number of byte to be read
 */
void SX1280HalReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t xsize);
/*!
 * \brief Returns the status of DIOs pins 返回DIOx引脚的状态
 *
 * \retval      dioStatus     A byte where each bit represents a DIO state:
 *                            [ DIOx | BUSY ]
 */
uint8_t SX1280HalGetDioStatus(void);

#endif