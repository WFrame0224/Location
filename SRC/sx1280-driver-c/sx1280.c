/**
 * @Description:本文件是2.4G文件的上层，属于硬件抽象层，用以实现射频定义的各种基本功能
 * @Filename：sx1280.c
 * @Author:王非
 * @Date：2018.08.28
 */ 

#include "sx1280.h"
#include "radio.h"
#include <string.h>
#include <stdio.h>
#include <hal.h>
/*!
 * \brief Radio registers definition
 *
 */
typedef struct
{
    uint16_t      Addr;                             //!< The address of the register
    uint8_t       Value;                            //!< The value of the register
}RadioRegisters_t;
/*!
 * \brief Radio hardware registers initialization definition
 */
// { Address, RegValue }
#define RADIO_INIT_REGISTERS_VALUE  { 0x00 }
/*!
 * \brief Radio hardware registers initialization
 */
const RadioRegisters_t xdata RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;
/*!
 * \brief Holds the internal operating mode of the radio
 */
static RadioOperatingModes_t xdata OperatingMode;

/*!
 * \brief Stores the current packet type set in the radio
 */
static RadioPacketTypes_t xdata PacketType;

/*!
 * \brief Stores the current LoRa bandwidth set in the radio
 */
static RadioLoRaBandwidths_t xdata LoRaBandwidth;

/*!
 * \brief Holds the polling state of the driver
 */
static bool xdata PollingMode;


//void SX1280OnDioIrq( void );
///*!
// * Hardware DIO IRQ callback initialization
// */
//DioIrqHandler *DioIrq[] = { SX1280OnDioIrq };



/*!
 * \brief Holds a flag raised on radio interrupt
 */
static bool xdata IrqState;

static RadioCallbacks_t* RadioCallbacks;

/*函数实现*************************************************************/
/*!
 * 计算小于32位的寄存器的两个补码 
 */
int32_t SX1280complement2( const uint32_t num, const uint8_t bitCnt )
{
    int32_t retVal = ( int32_t )num;
    if( num >= 2<<( bitCnt - 2 ) )
    {
        retVal -= 2<<( bitCnt - 1 );
    }
    return retVal;
}

void SX1280Init( RadioCallbacks_t *callbacks )
{
    RadioCallbacks = callbacks;

    //SX1280HalInit( DioIrq );
	SX1280HalInit();
}

void SX1280SetRegistersDefault( void )
{
    int16_t i = 0;
	for(i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SX1280HalWriteRegister( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }
}
RadioStatus_t SX1280GetStatus( void )
{
    uint8_t stat = 0;
    RadioStatus_t status;

    SX1280HalReadCommand( RADIO_GET_STATUS, ( uint8_t * )&stat, 1 );
    status.Value = stat;
    return status;
}

RadioOperatingModes_t SX1280GetOpMode( void )
{
    return OperatingMode;
}

/*
void SX1280SetSleep( SleepParams_t sleepConfig )
{
    uint8_t sleep = ( sleepConfig.WakeUpRTC << 3 ) |
                    ( sleepConfig.InstructionRamRetention << 2 ) |
                    ( sleepConfig.DataBufferRetention << 1 ) |
                    ( sleepConfig.DataRamRetention );

    OperatingMode = MODE_SLEEP;
    SX1280HalWriteCommand( RADIO_SET_SLEEP, &sleep, 1 );
}
*/
void SX1280SetStandby( RadioStandbyModes_t standbyConfig )
{
    SX1280HalWriteCommand( RADIO_SET_STANDBY, ( uint8_t* )&standbyConfig, 1 );
    if( standbyConfig == STDBY_RC )
    {
        OperatingMode = MODE_STDBY_RC;
    }
    else
    {
        OperatingMode = MODE_STDBY_XOSC;
    }
}

void SX1280SetFs( void )
{
    SX1280HalWriteCommand( RADIO_SET_FS, 0, 0 );
    OperatingMode = MODE_FS;
}
void SX1280SetTx( TickTime_t timeout )
{
    uint8_t buf[3];
    buf[0] = timeout.Step;
    buf[1] = ( uint8_t )( ( timeout.NbSteps >> 8 ) & 0x00FF );
    buf[2] = ( uint8_t )( timeout.NbSteps & 0x00FF );

    SX1280ClearIrqStatus( IRQ_RADIO_ALL );

    // If the radio is doing ranging operations, then apply the specific calls
    // prior to SetTx
    if( SX1280GetPacketType( ) == PACKET_TYPE_RANGING )
    {
        SX1280SetRangingRole( RADIO_RANGING_ROLE_MASTER );
    }
    SX1280HalWriteCommand( RADIO_SET_TX, buf, 3 );
    OperatingMode = MODE_TX;
}
void SX1280SetRx( TickTime_t timeout )
{
    uint8_t buf[3];
    buf[0] = timeout.Step;
    buf[1] = ( uint8_t )( ( timeout.NbSteps >> 8 ) & 0x00FF );
    buf[2] = ( uint8_t )( timeout.NbSteps & 0x00FF );

    SX1280ClearIrqStatus( IRQ_RADIO_ALL );

    // If the radio is doing ranging operations, then apply the specific calls
    // prior to SetRx
    if( SX1280GetPacketType( ) == PACKET_TYPE_RANGING )
    {
        SX1280SetRangingRole( RADIO_RANGING_ROLE_SLAVE );
    }
    SX1280HalWriteCommand( RADIO_SET_RX, buf, 3 );
    OperatingMode = MODE_RX;
}

/*
void SX1280SetRxDutyCycle( RadioTickSizes_t Step, uint16_t NbStepRx, uint16_t RxNbStepSleep )
{
    uint8_t buf[5];

    buf[0] = Step;
    buf[1] = ( uint8_t )( ( NbStepRx >> 8 ) & 0x00FF );
    buf[2] = ( uint8_t )( NbStepRx & 0x00FF );
    buf[3] = ( uint8_t )( ( RxNbStepSleep >> 8 ) & 0x00FF );
    buf[4] = ( uint8_t )( RxNbStepSleep & 0x00FF );
    SX1280HalWriteCommand( RADIO_SET_RXDUTYCYCLE, buf, 5 );
    OperatingMode = MODE_RX;
}


void SX1280SetCad( void )
{
    SX1280HalWriteCommand( RADIO_SET_CAD, 0, 0 );
    OperatingMode = MODE_CAD;
}
*/
/*
void SX1280SetTxContinuousWave( void )
{
    SX1280HalWriteCommand( RADIO_SET_TXCONTINUOUSWAVE, 0, 0 );
}
*/
void SX1280SetPacketType( RadioPacketTypes_t packetType )
{
    // Save packet type internally to avoid questioning the radio
    PacketType = packetType;

    SX1280HalWriteCommand( RADIO_SET_PACKETTYPE, ( uint8_t* )&packetType, 1 );
}
RadioPacketTypes_t SX1280GetPacketType( void )
{
    return PacketType;
}
void SX1280SetRfFrequency( uint32_t frequency )
{
    uint8_t buf[3];
    uint32_t freq = 0;

    freq = ( uint32_t )( ( float )frequency / ( float )FREQ_STEP );
    buf[0] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( freq & 0xFF );
    SX1280HalWriteCommand( RADIO_SET_RFFREQUENCY, buf, 3 );
}
void SX1280SetTxParams( int8_t power, RadioRampTimes_t rampTime )
{
    uint8_t buf[2];

    // The power value to send on SPI/UART is in the range [0..31] and the
    // physical output power is in the range [-18..13]dBm
    buf[0] = power + 18;
    buf[1] = ( uint8_t )rampTime;
    SX1280HalWriteCommand( RADIO_SET_TXPARAMS, buf, 2 );
}
/*
void SX1280SetCadParams( RadioLoRaCadSymbols_t cadSymbolNum )
{
    SX1280HalWriteCommand( RADIO_SET_CADPARAMS, ( uint8_t* )&cadSymbolNum, 1 );
    OperatingMode = MODE_CAD;
}
*/
void SX1280SetBufferBaseAddresses( uint8_t txBaseAddress, uint8_t rxBaseAddress )
{
    uint8_t buf[2];

    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    SX1280HalWriteCommand( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );
}
void SX1280SetModulationParams( ModulationParams_t *modulationParams )
{
    uint8_t buf[3];

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if( PacketType != modulationParams->PacketType )
    {
        SX1280SetPacketType( modulationParams->PacketType );
    }

    switch( modulationParams->PacketType )
    {
        case PACKET_TYPE_LORA:
        case PACKET_TYPE_RANGING:
            buf[0] = modulationParams->Params.LoRa.SpreadingFactor;
            buf[1] = modulationParams->Params.LoRa.Bandwidth;
            buf[2] = modulationParams->Params.LoRa.CodingRate;
            LoRaBandwidth = modulationParams->Params.LoRa.Bandwidth;
            break;

        case PACKET_TYPE_NONE:
            buf[0] = 0x00;
            buf[1] = 0x00;
            buf[2] = 0x00;
            break;
    }
    SX1280HalWriteCommand( RADIO_SET_MODULATIONPARAMS, buf, 3 );
	
	switch(modulationParams->Params.LoRa.SpreadingFactor)
	{	
		case LORA_SF5:
		case LORA_SF6:
			SX1280HalWriteRegister(0x925,0x1E);
			break;
		case LORA_SF7:
		case LORA_SF8:
			SX1280HalWriteRegister(0x925,0x37);
			break;
		case LORA_SF9:
		case LORA_SF10:
		case LORA_SF11:
		case LORA_SF12:
			SX1280HalWriteRegister(0x925,0x32);
			break;
		default:
			break;
	}
}
void SX1280SetPacketParams( PacketParams_t *packetParams )
{
    uint8_t buf[7];

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if( PacketType != packetParams->PacketType )
    {
        SX1280SetPacketType( packetParams->PacketType );
    }

    switch( packetParams->PacketType )
    {
        case PACKET_TYPE_LORA:
        case PACKET_TYPE_RANGING:
            buf[0] = packetParams->Params.LoRa.PreambleLength;
            buf[1] = packetParams->Params.LoRa.HeaderType;
            buf[2] = packetParams->Params.LoRa.PayloadLength;
            buf[3] = packetParams->Params.LoRa.CrcMode;
            buf[4] = packetParams->Params.LoRa.InvertIQ;
            buf[5] = 0x00;
            buf[6] = 0x00;
            break;
		
        case PACKET_TYPE_NONE:
            buf[0] = 0x00;
            buf[1] = 0x00;
            buf[2] = 0x00;
            buf[3] = 0x00;
            buf[4] = 0x00;
            buf[5] = 0x00;
            buf[6] = 0x00;
            break;
    }
    SX1280HalWriteCommand( RADIO_SET_PACKETPARAMS, buf, 7 );
}
void SX1280GetRxBufferStatus( uint8_t *payloadLength, uint8_t *rxStartBufferPointer )
{
    uint8_t status[2];

    SX1280HalReadCommand( RADIO_GET_RXBUFFERSTATUS, status, 2 );

    // In case of LORA fixed header, the payloadLength is obtained by reading
    // the register REG_LR_PAYLOADLENGTH
    if( ( SX1280GetPacketType( ) == PACKET_TYPE_LORA ) && ( SX1280HalReadRegister( REG_LR_PACKETPARAMS ) >> 7 == 1 ) )
    {
        *payloadLength = SX1280HalReadRegister( REG_LR_PAYLOADLENGTH );
    }
    else if( SX1280GetPacketType( ) == PACKET_TYPE_BLE )
    {
        // In the case of BLE, the msize returned in status[0] do not include the 2-byte length PDU header
        // so it is added there
        *payloadLength = status[0] + 2;
    }
    else
    {
        *payloadLength = status[0];
    }

    *rxStartBufferPointer = status[1];
}

void SX1280GetPacketStatus( PacketStatus_t *pktStatus )
{
    uint8_t status[5];

    SX1280HalReadCommand( RADIO_GET_PACKETSTATUS, status, 5 );

    pktStatus->packetType = SX1280GetPacketType( );
    switch( pktStatus->packetType )
    {
        case PACKET_TYPE_LORA:
        case PACKET_TYPE_RANGING:
            pktStatus->Params.LoRa.RssiPkt = -status[0] / 2;
            ( status[1] < 128 ) ? ( pktStatus->Params.LoRa.SnrPkt = status[1] / 4 ) : ( pktStatus->Params.LoRa.SnrPkt = ( ( status[1] - 256 ) /4 ) );

            pktStatus->Params.LoRa.ErrorStatus.SyncError = ( status[2] >> 6 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.LengthError = ( status[2] >> 5 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.CrcError = ( status[2] >> 4 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.AbortError = ( status[2] >> 3 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.HeaderReceived = ( status[2] >> 2 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.PacketReceived = ( status[2] >> 1 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

            pktStatus->Params.LoRa.TxRxStatus.RxNoAck = ( status[3] >> 5 ) & 0x01;
            pktStatus->Params.LoRa.TxRxStatus.PacketSent = status[3] & 0x01;

            pktStatus->Params.LoRa.SyncAddrStatus = status[4] & 0x07;
            break;

        
        case PACKET_TYPE_NONE:
            // In that specific case, we set everything in the pktStatus to zeros
            // and reset the packet type accordingly
            memset( pktStatus, 0, sizeof( PacketStatus_t ) );
            pktStatus->packetType = PACKET_TYPE_NONE;
            break;
    }
}

int8_t SX1280GetRssiInst( void )
{
    uint8_t raw = 0;

    SX1280HalReadCommand( RADIO_GET_RSSIINST, &raw, 1 );

//    return ( int8_t )( -raw / 2 );
	return raw;
}
void SX1280SetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
    uint8_t buf[8];

    buf[0] = ( uint8_t )( ( irqMask >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( irqMask & 0x00FF );
    buf[2] = ( uint8_t )( ( dio1Mask >> 8 ) & 0x00FF );
    buf[3] = ( uint8_t )( dio1Mask & 0x00FF );
    buf[4] = ( uint8_t )( ( dio2Mask >> 8 ) & 0x00FF );
    buf[5] = ( uint8_t )( dio2Mask & 0x00FF );
    buf[6] = ( uint8_t )( ( dio3Mask >> 8 ) & 0x00FF );
    buf[7] = ( uint8_t )( dio3Mask & 0x00FF );
    SX1280HalWriteCommand( RADIO_SET_DIOIRQPARAMS, buf, 8 );
}

uint16_t SX1280GetIrqStatus( void )
{
    uint8_t irqStatus[2];

    SX1280HalReadCommand( RADIO_GET_IRQSTATUS, irqStatus, 2 );

    return ( irqStatus[0] << 8 ) | irqStatus[1];
}

void SX1280ClearIrqStatus( uint16_t irq )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( ( uint16_t )irq >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( ( uint16_t )irq & 0x00FF );
    SX1280HalWriteCommand( RADIO_CLR_IRQSTATUS, buf, 2 );
}

/*
void SX1280Calibrate( CalibrationParams_t calibParam )
{
    uint8_t cal = ( calibParam.ADCBulkPEnable << 5 ) |
                  ( calibParam.ADCBulkNEnable << 4 ) |
                  ( calibParam.ADCPulseEnable << 3 ) |
                  ( calibParam.PLLEnable << 2 ) |
                  ( calibParam.RC13MEnable << 1 ) |
                  ( calibParam.RC64KEnable );

    SX1280HalWriteCommand( RADIO_CALIBRATE, &cal, 1 );
}
*/
void SX1280SetRegulatorMode( RadioRegulatorModes_t mode )
{
    SX1280HalWriteCommand( RADIO_SET_REGULATORMODE, ( uint8_t* )&mode, 1 );
}
/*
void SX1280SetSaveContext( void )
{
    SX1280HalWriteCommand( RADIO_SET_SAVECONTEXT, 0, 0 );
}

void SX1280SetAutoTx( uint16_t time )
{
    uint16_t compensatedTime = time - ( uint16_t )AUTO_RX_TX_OFFSET;
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( compensatedTime >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( compensatedTime & 0x00FF );
    SX1280HalWriteCommand( RADIO_SET_AUTOTX, buf, 2 );
}

void SX1280SetAutoFS( uint8_t enable )
{
    SX1280HalWriteCommand( RADIO_SET_AUTOFS, &enable, 1 );
}

void SX1280SetLongPreamble( uint8_t enable )
{
    SX1280HalWriteCommand( RADIO_SET_LONGPREAMBLE, &enable, 1 );
}
*/
void SX1280SetPayload( uint8_t *buffer, uint8_t msize )
{
    SX1280HalWriteBuffer( 0x00, buffer, msize );
}
uint8_t SX1280GetPayload( uint8_t *buffer, uint8_t *msize , uint8_t maxmsize )
{
    uint8_t offset;

    SX1280GetRxBufferStatus( msize, &offset );
    if( *msize > maxmsize )
    {
        return 1;
    }
    SX1280HalReadBuffer( offset, buffer, *msize );
    return 0;
}

void SX1280SendPayload( uint8_t *payload, uint8_t msize, TickTime_t timeout )
{
    SX1280SetPayload( payload, msize );
    SX1280SetTx( timeout );
}
/*
void SX1280SetRangingIdLength( RadioRangingIdCheckLengths_t length )
{
    switch( SX1280GetPacketType( ) )
    {
        case PACKET_TYPE_RANGING:
            SX1280HalWriteRegister( REG_LR_RANGINGIDCHECKLENGTH, ( ( ( ( uint8_t )length ) & 0x03 ) << 6 ) | ( SX1280HalReadRegister( REG_LR_RANGINGIDCHECKLENGTH ) & 0x3F ) );
            break;

        default:
            break;
    }
}
*/
/*
void SX1280SetDeviceRangingAddress( uint32_t address )
{
    uint8_t addrArray[] = {0x00,0x00,0x00,0x00};
	addrArray[0] =  address >> 24;
	addrArray[1] =  address >> 16;
	addrArray[2] =  address >> 8;
	addrArray[3] =  address >> 0;

    switch( SX1280GetPacketType( ) )
    {
        case PACKET_TYPE_RANGING:
            SX1280HalWriteRegisters( REG_LR_DEVICERANGINGADDR, addrArray, 4 );
            break;

        default:
            break;
    }
}

void SX1280SetRangingRequestAddress( uint32_t address )
{
    uint8_t addrArray[] = {0x00,0x00,0x00,0x00};
	addrArray[0] =  address >> 24;
	addrArray[1] =  address >> 16;
	addrArray[2] =  address >> 8;
	addrArray[3] =  address >> 0;

    switch( SX1280GetPacketType( ) )
    {
        case PACKET_TYPE_RANGING:
            SX1280HalWriteRegisters( REG_LR_REQUESTRANGINGADDR, addrArray, 4 );
            break;

        default:
            break;
    }
}


float SX1280GetRangingResult( RadioRangingResultTypes_t resultType )
{
    uint32_t valLsb = 0;
    float val = 0.0;

    switch( SX1280GetPacketType( ) )
    {
        case PACKET_TYPE_RANGING:
            SX1280SetStandby( STDBY_XOSC );
            SX1280HalWriteRegister( 0x97F, SX1280HalReadRegister( 0x97F ) | ( 1 << 1 ) ); // enable LORA modem clock
            SX1280HalWriteRegister( REG_LR_RANGINGRESULTCONFIG, ( SX1280HalReadRegister( REG_LR_RANGINGRESULTCONFIG ) & MASK_RANGINGMUXSEL ) | ( ( ( ( uint8_t )resultType ) & 0x03 ) << 4 ) );
            valLsb = ( ( SX1280HalReadRegister( REG_LR_RANGINGRESULTBASEADDR ) << 16 ) | ( SX1280HalReadRegister( REG_LR_RANGINGRESULTBASEADDR + 1 ) << 8 ) | ( SX1280HalReadRegister( REG_LR_RANGINGRESULTBASEADDR + 2 ) ) );
            SX1280SetStandby( STDBY_RC );

            // Convertion from LSB to distance. For explanation on the formula, refer to Datasheet of SX1280
            switch( resultType )
            {
                case RANGING_RESULT_RAW:
                    // Convert the ranging LSB to distance in meter
                    val = ( float )SX1280complement2( valLsb, 24 ) / ( float )SX1280GetLoRaBandwidth( ) * 36621.09375;
                    break;

                case RANGING_RESULT_AVERAGED:
                case RANGING_RESULT_DEBIASED:
                case RANGING_RESULT_FILTERED:
                    val = ( float )valLsb * 20.0 / 100.0;
                    break;

                default:
                    val = 0.0;
            }
            break;

        default:
            break;
    }
    return val;
}



void SX1280SetRangingCalibration( uint16_t cal )
{
    switch( SX1280GetPacketType( ) )
    {
        case PACKET_TYPE_RANGING:
            SX1280HalWriteRegister( REG_LR_RANGINGRERXTXDELAYCAL, ( uint8_t )( ( cal >> 8 ) & 0xFF ) );
            SX1280HalWriteRegister( REG_LR_RANGINGRERXTXDELAYCAL + 1, ( uint8_t )( ( cal ) & 0xFF ) );
            break;

        default:
            break;
    }
}
*/

void SX1280RangingClearFilterResult( void )
{
    uint8_t regVal = SX1280HalReadRegister( REG_LR_RANGINGRESULTCLEARREG );

    // To clear result, set bit 5 to 1 then to 0
    SX1280HalWriteRegister( REG_LR_RANGINGRESULTCLEARREG, regVal | ( 1 << 5 ) );
    SX1280HalWriteRegister( REG_LR_RANGINGRESULTCLEARREG, regVal & ( ~( 1 << 5 ) ) );
}


void SX1280RangingSetFilterNumSamples( uint8_t num )
{
    // Silently set 8 as minimum value
    SX1280HalWriteRegister( REG_LR_RANGINGFILTERWINDOWSIZE, ( num < DEFAULT_RANGING_FILTER_SIZE ) ? DEFAULT_RANGING_FILTER_SIZE : num );
}


void SX1280SetRangingRole( RadioRangingRoles_t role )
{
    uint8_t buf[1];

    buf[0] = role;
    SX1280HalWriteCommand( RADIO_SET_RANGING_ROLE, &buf[0], 1 );
}

float SX1280GetFrequencyError( )
{
    uint8_t efeRaw[3] = {0};
    uint32_t efe = 0;
    float efeHz = 0.0;

    switch( SX1280GetPacketType( ) )
    {
        case PACKET_TYPE_LORA:
        case PACKET_TYPE_RANGING:
            efeRaw[0] = SX1280HalReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB );
            efeRaw[1] = SX1280HalReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 1 );
            efeRaw[2] = SX1280HalReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 2 );
            efe = ( efeRaw[0]<<16 ) | ( efeRaw[1]<<8 ) | efeRaw[2];
            efe &= REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK;

            efeHz = 1.55 * ( float )SX1280complement2( efe, 20 ) / ( 1600.0 / ( float )SX1280GetLoRaBandwidth( ) * 1000.0 );
            break;

        case PACKET_TYPE_NONE:
        case PACKET_TYPE_BLE:
        case PACKET_TYPE_FLRC:
        case PACKET_TYPE_GFSK:
            break;
    }

    return efeHz;
}

void SX1280SetPollingMode(void)
{
    PollingMode = true;
}
int32_t SX1280GetLoRaBandwidth( )
{
    int32_t bwValue = 0;

    switch( LoRaBandwidth )
    {
        case LORA_BW_0200:
            bwValue = 203125;
            break;

        case LORA_BW_0400:
            bwValue = 406250;
            break;

        case LORA_BW_0800:
            bwValue = 812500;
            break;

        case LORA_BW_1600:
            bwValue = 1625000;
            break;

        default:
            bwValue = 0;
    }
    return bwValue;
}

void SX1280SetInterruptMode( void )
{
    PollingMode = false;
}
/*
void SX1280OnDioIrq( void )
{
//     * When polling mode is activated, it is up to the application to call
//     * ProcessIrqs( ). Otherwise, the driver automatically calls ProcessIrqs( )
//     * on radio interrupt.
    if( PollingMode == true )
    {
        IrqState = true;
    }
    else
    {
        SX1280ProcessIrqs( );
    }
}
*/
/*
void SX1280ProcessIrqs( void )
{
	RadioPacketTypes_t packetType = PACKET_TYPE_NONE;
	uint16_t irqRegs = 0;
	
    if( SX1280GetOpMode( ) == MODE_SLEEP )
    {
        return; // DIO glitch on V2b :-)
    }

    if( PollingMode == true )
    {
        if( IrqState == true )
        {
            EA = 0;
            IrqState = false;
            EA = 1;
        }
        else
        {
            return;
        }
    }

    packetType = SX1280GetPacketType( );
    irqRegs = SX1280GetIrqStatus( );
    SX1280ClearIrqStatus( IRQ_RADIO_ALL );

    switch( packetType )
    {
        case PACKET_TYPE_LORA:
            switch( OperatingMode )
            {
                case MODE_RX:
                    if( ( irqRegs & IRQ_RX_DONE ) == IRQ_RX_DONE )
                    {
                        if( ( irqRegs & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR )
                        {
                            if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->rxError != 0x00 ) )
                            {
                                RadioCallbacks->rxError( IRQ_CRC_ERROR_CODE );
                            }
                        }
                        else
                        {
                            if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->rxDone != 0x00 ) )
                            {
                                RadioCallbacks->rxDone( );
                            }
                        }
                    }
                    if( ( irqRegs & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID )
                    {
                        if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->rxHeaderDone != 0x00 ) )
                        {
                            RadioCallbacks->rxHeaderDone( );
                        }
                    }
                    if( ( irqRegs & IRQ_HEADER_ERROR ) == IRQ_HEADER_ERROR )
                    {
                        if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->rxError != 0x00 ) )
                        {
                            RadioCallbacks->rxError( IRQ_HEADER_ERROR_CODE );
                        }
                    }
                    if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
                    {
                        if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->rxTimeout != 0x00 ) )
                        {
                            RadioCallbacks->rxTimeout( );
                        }
                    }
                    if( ( irqRegs & IRQ_RANGING_SLAVE_REQUEST_DISCARDED ) == IRQ_RANGING_SLAVE_REQUEST_DISCARDED )
                    {
                        if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->rxError != 0x00 ) )
                        {
                            RadioCallbacks->rxError( IRQ_RANGING_ON_LORA_ERROR_CODE );
                        }
                    }
                    break;
                case MODE_TX:
                    if( ( irqRegs & IRQ_TX_DONE ) == IRQ_TX_DONE )
                    {
                        if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->txDone != 0x00 ) )
                        {
                            RadioCallbacks->txDone( );
                        }
                    }
                    if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
                    {
                        if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->txTimeout != 0x00 ) )
                        {
                            RadioCallbacks->txTimeout( );
                        }
                    }
                    break;
                case MODE_CAD:
                    if( ( irqRegs & IRQ_CAD_DONE ) == IRQ_CAD_DONE )
                    {
                        if( ( irqRegs & IRQ_CAD_ACTIVITY_DETECTED ) == IRQ_CAD_ACTIVITY_DETECTED )
                        {
                            if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->cadDone != 0x00 ) )
                            {
                                RadioCallbacks->cadDone( true );
                            }
                        }
                        else
                        {
                            if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->cadDone != 0x00 ) )
                            {
                                RadioCallbacks->cadDone( false );
                            }
                        }
                    }
                    else if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
                    {
                        if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->rxTimeout != 0x00 ) )
                        {
                            RadioCallbacks->rxTimeout( );
                        }
                    }
                    break;
                default:
                    // Unexpected IRQ: silently returns
                    break;
            }
            break;
        case PACKET_TYPE_RANGING:
            switch( OperatingMode )
            {
                // MODE_RX indicates an IRQ on the Slave side
                case MODE_RX:
                    if( ( irqRegs & IRQ_RANGING_SLAVE_REQUEST_DISCARDED ) == IRQ_RANGING_SLAVE_REQUEST_DISCARDED )
                    {
                        if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->rangingDone != 0x00 ) )
                        {
                            RadioCallbacks->rangingDone( IRQ_RANGING_SLAVE_ERROR_CODE );
                        }
                    }
                    if( ( irqRegs & IRQ_RANGING_SLAVE_REQUEST_VALID ) == IRQ_RANGING_SLAVE_REQUEST_VALID )
                    {
                        if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->rangingDone != 0x00 ) )
                        {
                            RadioCallbacks->rangingDone( IRQ_RANGING_SLAVE_VALID_CODE );
                        }
                    }
                    if( ( irqRegs & IRQ_RANGING_SLAVE_RESPONSE_DONE ) == IRQ_RANGING_SLAVE_RESPONSE_DONE )
                    {
                        if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->rangingDone != 0x00 ) )
                        {
                            RadioCallbacks->rangingDone( IRQ_RANGING_SLAVE_VALID_CODE );
                        }
                    }
                    if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
                    {
                        if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->rangingDone != 0x00 ) )
                        {
                            RadioCallbacks->rangingDone( IRQ_RANGING_SLAVE_ERROR_CODE );
                        }
                    }
                    if( ( irqRegs & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID )
                    {
                        if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->rxHeaderDone != 0x00 ) )
                        {
                            RadioCallbacks->rxHeaderDone( );
                        }
                    }
                    if( ( irqRegs & IRQ_HEADER_ERROR ) == IRQ_HEADER_ERROR )
                    {
                        if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->rxError != 0x00 ) )
                        {
                            RadioCallbacks->rxError( IRQ_HEADER_ERROR_CODE );
                        }
                    }
                    break;
                // MODE_TX indicates an IRQ on the Master side
                case MODE_TX:
                    if( ( irqRegs & IRQ_RANGING_MASTER_RESULT_TIMEOUT ) == IRQ_RANGING_MASTER_RESULT_TIMEOUT )
                    {
                        if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->rangingDone != 0x00 ) )
                        {
                            RadioCallbacks->rangingDone( IRQ_RANGING_MASTER_ERROR_CODE );
                        }
                    }
                    if( ( irqRegs & IRQ_RANGING_MASTER_RESULT_VALID ) == IRQ_RANGING_MASTER_RESULT_VALID )
                    {
                        if( ( RadioCallbacks != 0x00 ) && ( RadioCallbacks->rangingDone != 0x00 ) )
                        {
                            RadioCallbacks->rangingDone( IRQ_RANGING_MASTER_VALID_CODE );
                        }
                    }
                    break;
                default:
                    // Unexpected IRQ: silently returns
                    break;
            }
            break;
        default:
            // Unexpected IRQ: silently returns
            break;
    }
}
*/