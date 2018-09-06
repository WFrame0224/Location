/**
 * @Description：本程序仅定义与SX1280底层相关的函数，以及控制命令
 * @FileName：radio.c
 * @Author：王非
 * @Date:2018.08.13
 */ 

#include "radio.h"
#include "function.h"
#include "spi.h"
#include <string.h>


/*!
 * \brief Define the buf_size of tx and rx hal buffers
 *
 * The Tx and Rx hal buffers are used for SPI communication to
 * store data to be sent/receive to/from the chip.
 *
 * \warning The application must ensure the maximal useful buf_size to be much lower
 *          than the MAX_HAL_BUFFER_SIZE
 */
#define MAX_HAL_BUFFER_SIZE     0x1FF  //0xFFF

#define IRQ_HIGH_PRIORITY       0

/*!
 * Radio driver structure initialization
 */
static uint8_t  halTxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};
static uint8_t  halRxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};

/* Functions-------------------------------- */
/*!
 * \brief Used to block execution waiting for low state on radio busy pin.
 *        Essentially used in SPI communications
 */

void SX1280HalWaitOnBusy( void )
{
    while(Radio_Busy == 1)
        ;
}
void SX1280HalInit()
{
    SpiInit();
    SX1280HalReset( );
    // 需要定义外部 DIO 功能脚的外部中断
}
/*
void SX1280HalIoIrqInit()
{
    // 此处为设置DIO脚为外部中断引脚，同其Demo不同
	// GpioSetIrq( RADIO_DIOx_PORT, RADIO_DIOx_PIN, IRQ_HIGH_PRIORITY, irqHandlers[0] );
}
*/
void SX1280HalReset( void )
{
    Delay_Xms( 20 );
    Radio_Rest = 0;
    Delay_Xms( 50 );
    Radio_Rest = 1;
    Delay_Xms( 20 );
}
/*
void SX1280HalClearInstructionRam( void )
{
    // Clearing the instruction RAM is writing 0x00s on every bytes of the
    // instruction RAM
	uint16_t index = 0;
    uint16_t halSize = 3 + IRAM_SIZE;
	
    halTxBuffer[0] = RADIO_WRITE_REGISTER;
    halTxBuffer[1] = ( IRAM_START_ADDRESS >> 8 ) & 0x00FF;
    halTxBuffer[2] = IRAM_START_ADDRESS & 0x00FF;
    for(index = 0; index < IRAM_SIZE; index++ )
    {
        halTxBuffer[3+index] = 0x00;
    }

    SX1280HalWaitOnBusy( );

    SpiSetNssLH(0);

    SpiIn( halTxBuffer, halSize );

    SpiSetNssLH(1);

    SX1280HalWaitOnBusy( );
}

void SX120HalWakeup( void )
{
	// 关闭中断，没有实现
	uint16_t halSize = 2;
	
	SpiSetNssLH(0);

    halTxBuffer[0] = RADIO_GET_STATUS;
    halTxBuffer[1] = 0x00;
    SpiIn( halTxBuffer, halSize );

    SpiSetNssLH(1);

    // Wait for chip to be ready.
    SX1280HalWaitOnBusy( );
	
	// 开启中断，没有实现
}
*/
void SX1280HalWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t buf_size )
{
    uint16_t halSize  = buf_size + 1;
    SX1280HalWaitOnBusy( );

    halTxBuffer[0] = command;
    memcpy( halTxBuffer + 1, ( uint8_t * )buffer, buf_size * sizeof( uint8_t ) );

	SpiSetNssLH(0);
	
    SpiIn( halTxBuffer, halSize );

    SpiSetNssLH(1);

    if( command != RADIO_SET_SLEEP )
    {
        SX1280HalWaitOnBusy( );
    }
}
void SX1280HalReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t xsize )
{
    uint16_t index = 0;
	uint16_t halSize = 2 + xsize;
    halTxBuffer[0] = command;
    halTxBuffer[1] = 0x00;
    for(index = 0; index < xsize; index++ )
    {
        halTxBuffer[2+index] = 0x00;
    }

    SX1280HalWaitOnBusy( );

    SpiSetNssLH(0);

    SpiInOut( halTxBuffer, halRxBuffer, halSize );

    memcpy( buffer, halRxBuffer + 2, xsize );

    SpiSetNssLH(1);

    SX1280HalWaitOnBusy( );
}
void SX1280HalWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t xsize )
{
    uint16_t halSize = xsize + 3;
    halTxBuffer[0] = RADIO_WRITE_REGISTER;
    halTxBuffer[1] = ( address & 0xFF00 ) >> 8;
    halTxBuffer[2] = address & 0x00FF;
    memcpy( halTxBuffer + 3, buffer, xsize );

    SX1280HalWaitOnBusy( );

    SpiSetNssLH(0);

    SpiIn( halTxBuffer, halSize );

    SpiSetNssLH(1);

    SX1280HalWaitOnBusy( );
}
void SX1280HalWriteRegister( uint16_t address, uint8_t value )
{
    SX1280HalWriteRegisters( address, &value, 1 );
}
void SX1280HalReadRegisters( uint16_t address, uint8_t *buffer, uint16_t xsize )
{
    uint16_t index = 0;
	uint16_t halSize = 4 + xsize;
    halTxBuffer[0] = RADIO_READ_REGISTER;
    halTxBuffer[1] = ( address & 0xFF00 ) >> 8;
    halTxBuffer[2] = address & 0x00FF;
    halTxBuffer[3] = 0x00;
    for(index = 0; index < xsize; index++ )
    {
        halTxBuffer[4+index] = 0x00;
    }

    SX1280HalWaitOnBusy( );

    SpiSetNssLH(0);

    SpiInOut( halTxBuffer, halRxBuffer, halSize );

    memcpy( buffer, halRxBuffer + 4, xsize );

    SpiSetNssLH(1);

    SX1280HalWaitOnBusy( );
}
uint8_t SX1280HalReadRegister( uint16_t address )
{
    uint8_t dat;

    SX1280HalReadRegisters( address, &dat, 1 );

    return dat;
}
void SX1280HalWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t xsize )
{
    uint16_t halSize = xsize + 2;
    halTxBuffer[0] = RADIO_WRITE_BUFFER;
    halTxBuffer[1] = offset;
    memcpy( halTxBuffer + 2, buffer, xsize );

    SX1280HalWaitOnBusy( );

    SpiSetNssLH(0);

    SpiIn( halTxBuffer, halSize );

    SpiSetNssLH(1);

    SX1280HalWaitOnBusy( );
}
void SX1280HalReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t xsize )
{
    uint16_t index = 0;
	uint16_t halSize = xsize + 3;
    halTxBuffer[0] = RADIO_READ_BUFFER;
    halTxBuffer[1] = offset;
    halTxBuffer[2] = 0x00;
    for(index = 0; index < xsize; index++ )
    {
        halTxBuffer[3+index] = 0x00;
    }

    SX1280HalWaitOnBusy( );

    SpiSetNssLH(0);

    SpiInOut( halTxBuffer, halRxBuffer, halSize );

    memcpy( buffer, halRxBuffer + 3, xsize);

    SpiSetNssLH(1);

    SX1280HalWaitOnBusy( );
}
/*
uint8_t SX1280HalGetDioStatus( void )
{
    // 此处可能有点问题,Demo 中仅仅使用了一个DIO脚，暂时不知道使用的是哪个脚
	return (((0xff | Radio_Dio3 | Radio_Dio2 | Radio_Dio1) << 1) | (0xff | Radio_Busy) << 0);
}
*/