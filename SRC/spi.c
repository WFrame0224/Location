#include "spi.h"

void SpiInit()
{
	SPDAT = 0;
	SPSTAT = 0xc0;
	SPCTL = 0xd2;
}

void SpiWrite(uint8_t dat)
{
	SPDAT = dat;
	while(!(SPSTAT&0x80))
	{
		;
	}
	SPSTAT = 0xc0;
}

uint8_t SpiWRead(uint8_t dat)
{
	SPDAT = dat;
	while(!(SPSTAT&0x80))
	{
		;
	}
	SPSTAT = 0xc0;
	dat = SPDAT;
	
	return dat;
}

/*!
 * @brief Sends txBuffer and receives rxBuffer
 *
 * @param [IN] txBuffer Byte to be sent
 * @param [OUT] rxBuffer Byte to be sent
 * @param [IN] size Byte to be sent
 */
void SpiIn( uint8_t *txBuffer, uint16_t tx_size )
{
	uint8_t index, temp;
	
	
//	SPI_NSS = 0;	// 拉低片选端，开始传送数据
	
	for(index = 0; index < tx_size; index ++)
	{
		temp = txBuffer[index];
		SpiWrite(temp);
	}
	
//	SPI_NSS = 1;	// 拉高片选端，数据传输结束
	

}
void SpiInOut( uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t tx_size )
{
	uint8_t index, temp;
	uint8_t ucResult = 0;
	
//	SPI_NSS = 0;	// 拉低片选端，开始传送数据
	
	for(index = 0; index < tx_size; index ++)
	{
        temp = txBuffer[index];
		
        ucResult = SpiWRead(temp);
		rxBuffer[index] = ucResult;		// 将读出的数据赋值给接收缓存区
		ucResult = 0;

	}
	
//	SPI_NSS = 1;	// 拉高片选端，数据传输结束
}
/**
 * 方便外部设置NSS引脚，进行SPI通信
 */ 
void SpiSetNssLH(uint8_t level)
{
	switch(level)
	{
		case 0:
			SPI_NSS = 0;
			break;
		case 1:
			SPI_NSS = 1;
			break;
		default:
			SPI_NSS = 1;
			break;
	}
}