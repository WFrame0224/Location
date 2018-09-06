#ifndef __SPI_H__
#define __SPI_H__

#include "hal.h"
#include "datatype.h"

/* 首先定义使用的引脚 */
sbit SPI_NSS 	= 	P1^2;
// sbit SPI_MOSI 	= 	P1^3;
// sbit SPI_MISO	=	P1^4;
// sbit SPI_SCK	=	P1^5;

void SpiInit();
void SpiWrite(uint8_t dat);
uint8_t SpiWRead(uint8_t dat);
void SpiIn( uint8_t *txBuffer, uint16_t tx_size );
void SpiInOut( uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t tx_size );
void SpiSetNssLH(uint8_t level);

#endif