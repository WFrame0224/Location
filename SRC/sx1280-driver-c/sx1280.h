/**
 * @Desription:本文件是sx1280.c的头文件，属于射频的硬件抽象层，完成一些射频相关的数据定义以及函数声明
 * @Filename:sx1280.h
 * @Author:王非
 * @Date:2018.8.28
 */ 

#ifndef __SX1280_H__
#define __SX1280_H__

#include "datatype.h"
#include <math.h>

/*!
 * \brief Hardware IO IRQ callback function definition
 */
typedef void ( DioIrqHandler )( void );

/*!
 * \brief Provides the frequency of the chip running on the radio and the frequency step
 *
 * \remark These defines are used for computing the frequency divider to set the RF frequency
 */
#define XTAL_FREQ                                   52000000
#define FREQ_STEP                                   ( ( float )( XTAL_FREQ / pow( 2.0, 18.0 ) ) )


/*!
 * \brief Compensation delay for SetAutoTx/Rx functions in microseconds
 */
#define AUTO_RX_TX_OFFSET                           33


/* Ranging 相关******************************************************/
/*!
 * \brief The address of the register holding the ranging id check length
 *
 * \remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGIDCHECKLENGTH                 0x0931
/*!
 * \brief The address of the register holding the device ranging id
 *
 * \remark Only used for packet type Ranging
 */
#define REG_LR_DEVICERANGINGADDR                    0x0916
/*!
 * \brief The address of the register holding the device ranging id
 *
 * \remark Only used for packet type Ranging
 */
#define REG_LR_REQUESTRANGINGADDR                   0x0912

/*!
 * \brief The address of the register holding ranging results configuration
 * and the corresponding mask
 *
 * \remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGRESULTCONFIG                  0x0924
#define MASK_RANGINGMUXSEL                          0xCF

/*!
 * \brief The address of the register holding the first byte of ranging results
 * Only used for packet type Ranging
 */
#define REG_LR_RANGINGRESULTBASEADDR                0x0961

/*!
 * \brief The address of the register allowing to read ranging results
 *
 * \remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGRESULTSFREEZE                 0x097F

/*!
 * \brief The address of the register holding the first byte of ranging calibration
 *
 * \remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGRERXTXDELAYCAL                0x092C

/*!
 *\brief The address of the register holding the ranging filter window size
 *
 * \remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGFILTERWINDOWSIZE              0x091E

/*!
 *\brief The address of the register to reset for clearing ranging filter
 *
 * \remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGRESULTCLEARREG                0x0923
/*!
 * \brief The default number of samples considered in built-in ranging filter
 */
#define DEFAULT_RANGING_FILTER_SIZE                 127
/******************************************************************/

/* LORA 相关 ******************************************************/
/*!
 * \brief The address of the register holding LORA packet parameters
 */
#define REG_LR_PACKETPARAMS                         0x903
/******************************************************************/
/*!
 * \brief The address of the register holding payload length
 *
 * \remark Do NOT try to read it directly. Use GetRxBuffer( ) instead.
 */
#define REG_LR_PAYLOADLENGTH                        0x901

/*!
 * \brief The address of the instruction RAM and its size
 */
#define IRAM_START_ADDRESS                          0x8000
#define IRAM_SIZE                                   0x4000

/*!
 * \brief The addresses of the registers holding SyncWords values
 *
 * \remark The addresses depends on the Packet Type in use, and not all
 *         SyncWords are available for every Packet Type
 */
#define REG_LR_SYNCWORDBASEADDRESS1                 0x09CE
#define REG_LR_SYNCWORDBASEADDRESS2                 0x09D3
#define REG_LR_SYNCWORDBASEADDRESS3                 0x09D8

/*!
 * \brief The MSB address and mask used to read the estimated frequency
 * error
 */
#define REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB        0x0954
#define REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK       0x0FFFFF

/*!
 * \brief Defines how many bit errors are tolerated in sync word detection
 */
#define REG_LR_SYNCWORDTOLERANCE                    0x09CD


/*!
 * \brief Structure describing the radio status
 */
typedef union
{
    /*!
     * \brief Structure of the radio status
     */
    struct
    {
        uint8_t CpuBusy   : 1;  //!< Flag for CPU radio busy
        uint8_t DmaBusy   : 1;  //!< Flag for DMA busy
        uint8_t CmdStatus : 3;  //!< Command status
        uint8_t ChipMode  : 3;  //!< Chip mode
    }Fields;

    /*!
     * \brief Serialized radio status
     */
    uint8_t Value;
}RadioStatus_t;
/*!
 * \brief Represents the states of the radio
 */
typedef enum
{
    RF_IDLE                                 = 0x00,         //!< The radio is idle
    RF_RX_RUNNING,                                          //!< The radio is in reception state
    RF_TX_RUNNING,                                          //!< The radio is in transmission state
    RF_CAD,                                                 //!< The radio is doing channel activity detection
}RadioStates_t;
/*!
 * \brief Represents the operating mode the radio is actually running
 */
typedef enum
{
    MODE_SLEEP                              = 0x00,         //! The radio is in sleep mode
    MODE_STDBY_RC,                                          //! The radio is in standby mode with RC oscillator
    MODE_STDBY_XOSC,                                        //! The radio is in standby mode with XOSC oscillator
    MODE_FS,                                                //! The radio is in frequency synthesis mode
    MODE_TX,                                                //! The radio is in transmit mode
    MODE_RX,                                                //! The radio is in receive mode
    MODE_CAD                                                //! The radio is in channel activity detection mode
}RadioOperatingModes_t;
/*!
 * \brief Declares the oscillator in use while in standby mode
 *
 * Using the STDBY_RC standby mode allow to reduce the energy consumption
 * STDBY_XOSC should be used for time critical applications
 */
typedef enum
{
    STDBY_RC                                = 0x00,
    STDBY_XOSC                              = 0x01,
}RadioStandbyModes_t;

/*!
 * \brief Declares the power regulation used to power the device
 *
 * This command allows the user to specify if DC-DC or LDO is used for power regulation.
 * Using only LDO implies that the Rx or Tx current is doubled
 */
typedef enum
{
    USE_LDO                               = 0x00,           //! Use LDO (default value)
    USE_DCDC                              = 0x01,           //! Use DCDC
}RadioRegulatorModes_t;
/*!
 * \brief Represents the possible packet type (i.e. modem) used
 */
typedef enum
{
    PACKET_TYPE_GFSK                        = 0x00,
    PACKET_TYPE_LORA,
    PACKET_TYPE_RANGING,
    PACKET_TYPE_FLRC,
    PACKET_TYPE_BLE,
    PACKET_TYPE_NONE                        = 0x0F,
}RadioPacketTypes_t;
/*!
 * \brief Represents the ramping time for power amplifier
 */
typedef enum
{
    RADIO_RAMP_02_US                        = 0x00,
    RADIO_RAMP_04_US                        = 0x20,
    RADIO_RAMP_06_US                        = 0x40,
    RADIO_RAMP_08_US                        = 0x60,
    RADIO_RAMP_10_US                        = 0x80,
    RADIO_RAMP_12_US                        = 0xA0,
    RADIO_RAMP_16_US                        = 0xC0,
    RADIO_RAMP_20_US                        = 0xE0,
}RadioRampTimes_t;
/*!
 * \brief Represents the number of symbols to be used for channel activity detection operation
 */
typedef enum
{
    LORA_CAD_01_SYMBOL                      = 0x00,
    LORA_CAD_02_SYMBOL                      = 0x20,
    LORA_CAD_04_SYMBOL                      = 0x40,
    LORA_CAD_08_SYMBOL                      = 0x60,
    LORA_CAD_16_SYMBOL                      = 0x80,
}RadioLoRaCadSymbols_t;
/*!
 * \brief Represents the possible spreading factor values in LORA packet types
 */
typedef enum
{
    LORA_SF5                                = 0x50,
    LORA_SF6                                = 0x60,
    LORA_SF7                                = 0x70,
    LORA_SF8                                = 0x80,
    LORA_SF9                                = 0x90,
    LORA_SF10                               = 0xA0,
    LORA_SF11                               = 0xB0,
    LORA_SF12                               = 0xC0,
}RadioLoRaSpreadingFactors_t;
/*!
 * \brief Represents the bandwidth values for LORA packet type
 */
typedef enum
{
    LORA_BW_0200                            = 0x34,
    LORA_BW_0400                            = 0x26,
    LORA_BW_0800                            = 0x18,
    LORA_BW_1600                            = 0x0A,
}RadioLoRaBandwidths_t;
/*!
 * \brief Represents the coding rate values for LORA packet type
 */
typedef enum
{
    LORA_CR_4_5                             = 0x01,
    LORA_CR_4_6                             = 0x02,
    LORA_CR_4_7                             = 0x03,
    LORA_CR_4_8                             = 0x04,
    LORA_CR_LI_4_5                          = 0x05,
    LORA_CR_LI_4_6                          = 0x06,
    LORA_CR_LI_4_7                          = 0x07,
}RadioLoRaCodingRates_t;
/*!
 * \brief Holds the packet length mode of a LORA packet type
 */
typedef enum
{
    LORA_PACKET_VARIABLE_LENGTH             = 0x00,         //!< The packet is on variable size, header included
    LORA_PACKET_FIXED_LENGTH                = 0x80,         //!< The packet is known on both sides, no header included in the packet
    LORA_PACKET_EXPLICIT                    = LORA_PACKET_VARIABLE_LENGTH,
    LORA_PACKET_IMPLICIT                    = LORA_PACKET_FIXED_LENGTH,
}RadioLoRaPacketLengthsModes_t;
/*!
 * \brief Represents the CRC mode for LORA packet type
 */
typedef enum
{
    LORA_CRC_ON                             = 0x20,         //!< CRC activated
    LORA_CRC_OFF                            = 0x00,         //!< CRC not used
}RadioLoRaCrcModes_t;
/*!
 * \brief Represents the IQ mode for LORA packet type
 */
typedef enum
{
    LORA_IQ_NORMAL                          = 0x40,
    LORA_IQ_INVERTED                        = 0x00,
}RadioLoRaIQModes_t;

/*!
 * \brief Represents the length of the ID to check in ranging operation
 */
typedef enum
{
    RANGING_IDCHECK_LENGTH_08_BITS          = 0x00,
    RANGING_IDCHECK_LENGTH_16_BITS,
    RANGING_IDCHECK_LENGTH_24_BITS,
    RANGING_IDCHECK_LENGTH_32_BITS,
}RadioRangingIdCheckLengths_t;

/*!
 * \brief Represents the result type to be used in ranging operation
 */
typedef enum
{
    RANGING_RESULT_RAW                      = 0x00,
    RANGING_RESULT_AVERAGED                 = 0x01,
    RANGING_RESULT_DEBIASED                 = 0x02,
    RANGING_RESULT_FILTERED                 = 0x03,
}RadioRangingResultTypes_t;
/*!
 * \brief Represents the interruption masks available for the radio
 *
 * \remark Note that not all these interruptions are available for all packet types
 */
typedef enum
{
    IRQ_RADIO_NONE                          = 0x0000,
    IRQ_TX_DONE                             = 0x0001,
    IRQ_RX_DONE                             = 0x0002,
    IRQ_SYNCWORD_VALID                      = 0x0004,
    IRQ_SYNCWORD_ERROR                      = 0x0008,
    IRQ_HEADER_VALID                        = 0x0010,
    IRQ_HEADER_ERROR                        = 0x0020,
    IRQ_CRC_ERROR                           = 0x0040,
    IRQ_RANGING_SLAVE_RESPONSE_DONE         = 0x0080,
    IRQ_RANGING_SLAVE_REQUEST_DISCARDED     = 0x0100,
    IRQ_RANGING_MASTER_RESULT_VALID         = 0x0200,
    IRQ_RANGING_MASTER_RESULT_TIMEOUT       = 0x0400,
    IRQ_RANGING_SLAVE_REQUEST_VALID         = 0x0800,
    IRQ_CAD_DONE                            = 0x1000,
    IRQ_CAD_ACTIVITY_DETECTED               = 0x2000,
    IRQ_RX_TX_TIMEOUT                       = 0x4000,
    IRQ_PREAMBLE_DETECTED                   = 0x8000,
    IRQ_RADIO_ALL                           = 0xFFFF,
}RadioIrqMasks_t;
/*!
 * \brief Represents the digital input/output of the radio
 * 此处不知道是否和具体的引脚对应
 */
typedef enum
{
    RADIO_DIO1                              = 0x02,
    RADIO_DIO2                              = 0x04,
    RADIO_DIO3                              = 0x08,
}RadioDios_t;
/*!
 * \brief Represents the tick size available for Rx/Tx timeout operations
 */
typedef enum
{
    RADIO_TICK_SIZE_0015_US                 = 0x00,
    RADIO_TICK_SIZE_0062_US                 = 0x01,
    RADIO_TICK_SIZE_1000_US                 = 0x02,
    RADIO_TICK_SIZE_4000_US                 = 0x03,
}RadioTickSizes_t;
/*!
 * \brief Represents the role of the radio during ranging operations
 */
typedef enum
{
    RADIO_RANGING_ROLE_SLAVE                = 0x00,
    RADIO_RANGING_ROLE_MASTER               = 0x01,
}RadioRangingRoles_t;
/*!
 * \brief Represents an amount of time measurable by the radio clock
 *
 * @code
 * Time = Step * NbSteps
 * Example:
 * Step = RADIO_TICK_SIZE_4000_US( 4 ms )
 * NbSteps = 1000
 * Time = 4e-3 * 1000 = 4 seconds
 * @endcode
 */
typedef struct TickTime_s
{
    RadioTickSizes_t Step;                                  //!< The step of ticktime
    /*!
     * \brief The number of steps for ticktime
     * Special values are:
     *     - 0x0000 for single mode
     *     - 0xFFFF for continuous mode
     */
    uint16_t NbSteps;
}TickTime_t;

/*!
* \brief RX_TX_CONTINUOUS and RX_TX_SINGLE are two particular values for TickTime.
* The former keep the radio in Rx or Tx mode, even after successfull reception
* or transmission. It should never generate Timeout interrupt.
* The later let the radio enought time to make one reception or transmission.
* No Timeout interrupt is generated, and the radio fall in StandBy mode after
* reception or transmission.
*/
#define RX_TX_CONTINUOUS ( TickTime_t ){ RADIO_TICK_SIZE_0015_US, 0xFFFF }
#define RX_TX_SINGLE     ( TickTime_t ){ RADIO_TICK_SIZE_0015_US, 0 }

/*===========================================================================*/
/*!
 * \brief The type describing the modulation parameters for every packet types
 */
typedef struct
{
    RadioPacketTypes_t                    PacketType;        //!< Packet to which the modulation parameters are referring to.
//    union
    struct
    { 
        /*!
         * \brief Holds the LORA modulation parameters
         *
         * LORA modulation is defined by Spreading Factor (SF), Bandwidth and Coding Rate
         */
        struct
        {
            RadioLoRaSpreadingFactors_t  SpreadingFactor;   //!< Spreading Factor for the LORA modulation
            RadioLoRaBandwidths_t        Bandwidth;         //!< Bandwidth for the LORA modulation
            RadioLoRaCodingRates_t       CodingRate;        //!< Coding rate for the LORA modulation
        }LoRa;
        // 还能定义其他模式
    }Params;                                                   //!< Holds the modulation parameters structure
}ModulationParams_t;
/*!
 * \brief Structure describing the error codes for callback functions
 */
typedef enum
{
    IRQ_HEADER_ERROR_CODE                   = 0x00,
    IRQ_SYNCWORD_ERROR_CODE,
    IRQ_CRC_ERROR_CODE,
    IRQ_RANGING_ON_LORA_ERROR_CODE,
}IrqErrorCode_t;
/*!
 * \brief Structure describing the ranging codes for callback functions
 */
typedef enum
{
    IRQ_RANGING_SLAVE_ERROR_CODE            = 0x00,
    IRQ_RANGING_SLAVE_VALID_CODE,
    IRQ_RANGING_MASTER_ERROR_CODE,
    IRQ_RANGING_MASTER_VALID_CODE,
}IrqRangingCode_t;
/*!
 * \brief The radio callbacks structure
 * Holds function pointers to be called on radio interrupts
 */
typedef struct
{
    void ( *txDone )( void );                       //!< Pointer to a function run on successful transmission
    void ( *rxDone )( void );                       //!< Pointer to a function run on successful reception
    void ( *rxSyncWordDone )( void );               //!< Pointer to a function run on successful SyncWord reception
    void ( *rxHeaderDone )( void );                 //!< Pointer to a function run on successful Header reception
    void ( *txTimeout )( void );                    //!< Pointer to a function run on transmission timeout
    void ( *rxTimeout )( void );                    //!< Pointer to a function run on reception timeout
    void ( *rxError )( IrqErrorCode_t errCode );    //!< Pointer to a function run on reception error
    void ( *rangingDone )( IrqRangingCode_t val );  //!< Pointer to a function run on ranging terminated
    void ( *cadDone )( bool cadFlag );              //!< Pointer to a function run on channel activity detected
}RadioCallbacks_t;
/*!
 * \brief The type describing the packet parameters for every packet types
 */
typedef struct
{
    RadioPacketTypes_t                    PacketType;        //!< Packet to which the packet parameters are referring to.
//    union
    struct
    {
        /*!
         * \brief Holds the LORA packet parameters
         */
        struct
        {
            uint8_t                       PreambleLength;    //!< The preamble length is the number of LORA symbols in the preamble. To set it, use the following formula @code Number of symbols = PreambleLength[3:0] * ( 2^PreambleLength[7:4] ) @endcode
            RadioLoRaPacketLengthsModes_t HeaderType;        //!< If the header is explicit, it will be transmitted in the LORA packet. If the header is implicit, it will not be transmitted
            uint8_t                       PayloadLength;     //!< Size of the payload in the LORA packet
            RadioLoRaCrcModes_t           CrcMode;           //!< Size of CRC block in LORA packet
            RadioLoRaIQModes_t            InvertIQ;          //!< Allows to swap IQ for LORA packet
        }LoRa;
        //此处还能扩展其他方式
    }Params;                                                 //!< Holds the packet parameters structure
}PacketParams_t;
/*!
 * \brief Represents the packet status for every packet type
 */
typedef struct
{
    RadioPacketTypes_t                    packetType;        //!< Packet to which the packet status are referring to.
    //union
    struct
    {
        struct
        {
            int8_t RssiPkt;                                 //!< The RSSI of the last packet
            int8_t SnrPkt;                                  //!< The SNR of the last packet
            struct
            {
                bool SyncError :1;                          //!< SyncWord error on last packet
                bool LengthError :1;                        //!< Length error on last packet
                bool CrcError :1;                           //!< CRC error on last packet
                bool AbortError :1;                         //!< Abort error on last packet
                bool HeaderReceived :1;                     //!< Header received on last packet
                bool PacketReceived :1;                     //!< Packet received
                bool PacketControlerBusy :1;                //!< Packet controller busy
            }ErrorStatus;                                   //!< The error status Byte
            struct
            {
                bool RxNoAck :1;                            //!< No acknowledgment received for Rx with variable length packets
                bool PacketSent :1;                         //!< Packet sent, only relevant in Tx mode
            }TxRxStatus;                                    //!< The Tx/Rx status Byte
            uint8_t SyncAddrStatus :3;                      //!< The id of the correlator who found the packet
        }LoRa;
        
    }Params;
}PacketStatus_t;
/*!
 * \brief Represents the Rx internal counters values when GFSK or LORA packet type is used
 */
typedef struct
{
    RadioPacketTypes_t                    packetType;       //!< Packet to which the packet status are referring to.
    //union
    struct
    {
        struct
        {
            uint16_t PacketReceived;                        //!< Number of received packets
            uint16_t CrcError;                              //!< Number of CRC errors
            uint16_t HeaderValid;                           //!< Number of valid headers
        }LoRa;
    }Params;
}RxCounter_t;
/*!
 * \brief Represents a calibration configuration
 */
typedef struct
{
    uint8_t RC64KEnable    : 1;                             //!< Calibrate RC64K clock
    uint8_t RC13MEnable    : 1;                             //!< Calibrate RC13M clock
    uint8_t PLLEnable      : 1;                             //!< Calibrate PLL
    uint8_t ADCPulseEnable : 1;                             //!< Calibrate ADC Pulse
    uint8_t ADCBulkNEnable : 1;                             //!< Calibrate ADC bulkN
    uint8_t ADCBulkPEnable : 1;                             //!< Calibrate ADC bulkP
}CalibrationParams_t;
/*!
 * \brief Represents a sleep mode configuration
 */
typedef struct
{
    uint8_t WakeUpRTC               : 1;                    //!< Get out of sleep mode if wakeup signal received from RTC
    uint8_t InstructionRamRetention : 1;                    //!< InstructionRam is conserved during sleep
    uint8_t DataBufferRetention     : 1;                    //!< Data buffer is conserved during sleep
    uint8_t DataRamRetention        : 1;                    //!< Data ram is conserved during sleep
}SleepParams_t;

/*=================================函数声明=======================================*/
/*!
 * \brief Compute the two's complement for a register of size lower than
 *        32bits
 *
 * \param [in]  num            The register to be two's complemented
 * \param [in]  bitCnt         The position of the sign bit
 */
static int32_t SX1280complement2( const uint32_t num, const uint8_t bitCnt );

/*!
 * \brief Initializes the radio driver
 */
void SX1280Init( RadioCallbacks_t *callbacks );
/*!
 * \brief Initializes the radio registers to the recommended default values
 */
void SX1280SetRegistersDefault( void );
/*!
 * \brief Gets the current radio status
 *
 * \retval      status        Radio status
 */
RadioStatus_t SX1280GetStatus( void );
/*!
 * \brief Gets the current Operation Mode of the Radio
 *
 * \retval      RadioOperatingModes_t last operating mode
 */
RadioOperatingModes_t SX1280GetOpMode( void );
/*!
 * \brief Sets the radio in sleep mode
 *
 * \param [in]  sleepConfig   The sleep configuration describing data
 *                            retention and RTC wake-up
 */
void SX1280SetSleep( SleepParams_t sleepConfig );
/*!
 * \brief Sets the radio in configuration mode
 *
 * \param [in]  mode          The standby mode to put the radio into
 */
void SX1280SetStandby( RadioStandbyModes_t mode );
/*!
 * \brief Sets the radio in FS mode
 */
void SX1280SetFs( void );
/*!
 * \brief Sets the radio in transmission mode
 *
 * \param [in]  timeout       Structure describing the transmission timeout value
 */
void SX1280SetTx( TickTime_t timeout );
/*!
 * \brief Sets the radio in reception mode
 *
 * \param [in]  timeout       Structure describing the reception timeout value
 */
void SX1280SetRx( TickTime_t timeout );
/*!
 * \brief Sets the Rx duty cycle management parameters
 *
 * \param [in]  rxTime        Structure describing reception timeout value
 * \param [in]  sleepTime     Structure describing sleep timeout value
 */
void SX1280SetRxDutyCycle( RadioTickSizes_t Step, uint16_t NbStepRx, uint16_t RxNbStepSleep );

/*!
 * \brief Sets the radio in CAD mode
 *
 * \see SX1280::SetCadParams
 */
void SX1280SetCad( void );
/*!
 * \brief Sets the radio in continuous wave transmission mode
 */
void SX1280SetTxContinuousWave( void );

/*!
 * \brief Sets the radio in continuous preamble transmission mode
 */
void SX1280SetTxContinuousPreamble( void );

/*!
 * \brief Sets the radio for the given protocol
 *
 * \param [in]  packetType    [PACKET_TYPE_GFSK, PACKET_TYPE_LORA,
 *                             PACKET_TYPE_RANGING, PACKET_TYPE_FLRC,
 *                             PACKET_TYPE_BLE]
 *
 * \remark This method has to be called before SetRfFrequency,
 *         SetModulationParams and SetPacketParams
 */
void SX1280SetPacketType( RadioPacketTypes_t packetType );

/*!
 * \brief Gets the current radio protocol
 *
 * \retval      packetType    [PACKET_TYPE_GFSK, PACKET_TYPE_LORA,
 *                             PACKET_TYPE_RANGING, PACKET_TYPE_FLRC,
 *                             PACKET_TYPE_BLE, PACKET_TYPE_NONE]
 */
RadioPacketTypes_t SX1280GetPacketType( void );

/*!
 * \brief Sets the RF frequency
 *
 * \param [in]  frequency     RF frequency [Hz]
 */
void SX1280SetRfFrequency( uint32_t frequency );

/*!
 * \brief Sets the transmission parameters
 *
 * \param [in]  power         RF output power [-18..13] dBm
 * \param [in]  rampTime      Transmission ramp up time
 */
void SX1280SetTxParams( int8_t power, RadioRampTimes_t rampTime );

/*!
 * \brief Sets the number of symbols to be used for Channel Activity
 *        Detection operation
 *
 * \param [in]  cadSymbolNum  The number of symbol to use for Channel Activity
 *                            Detection operations [LORA_CAD_01_SYMBOL, LORA_CAD_02_SYMBOL,
 *                            LORA_CAD_04_SYMBOL, LORA_CAD_08_SYMBOL, LORA_CAD_16_SYMBOL]
 */
void SX1280SetCadParams( RadioLoRaCadSymbols_t cadSymbolNum );

/*!
 * \brief Sets the data buffer base address for transmission and reception
 *
 * \param [in]  txBaseAddress Transmission base address
 * \param [in]  rxBaseAddress Reception base address
 */
void SX1280SetBufferBaseAddresses( uint8_t txBaseAddress, uint8_t rxBaseAddress );

/*!
 * \brief Set the modulation parameters
 *
 * \param [in]  modParams     A structure describing the modulation parameters
 */
void SX1280SetModulationParams( ModulationParams_t *modParams );

/*!
 * \brief Sets the packet parameters
 *
 * \param [in]  packetParams  A structure describing the packet parameters
 */
void SX1280SetPacketParams( PacketParams_t *packetParams );

/*!
 * \brief Gets the last received packet buffer status
 *
 * \param [out] payloadLength Last received packet payload length
 * \param [out] rxStartBuffer Last received packet buffer address pointer
 */
void SX1280GetRxBufferStatus( uint8_t *payloadLength, uint8_t *rxStartBuffer );

/*!
 * \brief Returns the instantaneous RSSI value for the last packet received
 *
 * \retval      rssiInst      Instantaneous RSSI
 */
int8_t SX1280GetRssiInst( void );

/*!
 * \brief   Sets the IRQ mask and DIO masks
 *
 * \param [in]  irqMask       General IRQ mask
 * \param [in]  dio1Mask      DIO1 mask
 * \param [in]  dio2Mask      DIO2 mask
 * \param [in]  dio3Mask      DIO3 mask
 */
void SX1280SetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask );

/*!
 * \brief Returns the current IRQ status
 *
 * \retval      irqStatus     IRQ status
 */
uint16_t SX1280GetIrqStatus( void );

/*!
 * \brief Clears the IRQs
 *
 * \param [in]  irq           IRQ(s) to be cleared
 */
void SX1280ClearIrqStatus( uint16_t irq );

/*!
 * \brief Calibrates the given radio block
 *
 * \param [in]  calibParam    The description of blocks to be calibrated
 */
void SX1280Calibrate( CalibrationParams_t calibParam );

/*!
 * \brief Sets the power regulators operating mode
 *
 * \param [in]  mode          [0: LDO, 1:DC_DC]
 */
void SX1280SetRegulatorMode( RadioRegulatorModes_t mode );

/*!
 * \brief Saves the current selected modem configuration into data RAM
 */
void SX1280SetSaveContext( void );

/*!
 * \brief Sets the chip to automatically send a packet after the end of a packet reception
 *
 * \remark The offset is automatically compensated inside the function
 *
 * \param [in]  time          The delay in us after which a Tx is done
 */
void SX1280SetAutoTx( uint16_t time );

/*!
 * \brief Sets the chip to automatically receive a packet after the end of a packet transmission
 *
 * \remark The offset is automatically compensated inside the function
 *
 * \param [in]  time          The delay in us after which a Rx is done
 */
void SX1280SetAutoFS( uint8_t enable );

/*!
 * \brief Enables or disables long preamble detection mode
 *
 * \param [in]  enable        [0: Disable, 1: Enable]
 */
void SX1280SetLongPreamble( uint8_t enable );

/*!
 * \brief Saves the payload to be send in the radio buffer
 *
 * \param [in]  payload       A pointer to the payload
 * \param [in]  size          The size of the payload
 */
void SX1280SetPayload( uint8_t *payload, uint8_t size );

/*!
 * \brief Reads the payload received. If the received payload is longer
 * than maxSize, then the method returns 1 and do not set size and payload.
 *
 * \param [out] payload       A pointer to a buffer into which the payload will be copied
 * \param [out] size          A pointer to the size of the payload received
 * \param [in]  maxSize       The maximal size allowed to copy into the buffer
 */
uint8_t SX1280GetPayload( uint8_t *payload, uint8_t *size, uint8_t maxSize );

/*!
 * \brief Sends a payload
 *
 * \param [in]  payload       A pointer to the payload to send
 * \param [in]  size          The size of the payload to send
 * \param [in]  timeout       The timeout for Tx operation
 */
void SX1280SendPayload( uint8_t *payload, uint8_t size, TickTime_t timeout );

/*!
 * \brief Sets the number of bits used to check that ranging request match ranging ID
 *
 * \param [in]  length        [0: 8 bits, 1: 16 bits,
 *                             2: 24 bits, 3: 32 bits]
 */
void SX1280SetRangingIdLength( RadioRangingIdCheckLengths_t length );

/*!
 * \brief Sets ranging device id
 *
 * \param [in]  address       Device address
 */
void SX1280SetDeviceRangingAddress( uint32_t address );

/*!
 * \brief Sets the device id to ping in a ranging request
 *
 * \param [in]  address       Address of the device to ping
 */
void SX1280SetRangingRequestAddress( uint32_t address );

/*!
 * \brief Return the ranging result value
 *
 * \param [in]  resultType    Specifies the type of result.
 *                            [0: RAW, 1: Averaged,
 *                             2: De-biased, 3:Filtered]
 *
 * \retval      ranging       The ranging measure filtered according to resultType [m]
 */
double SX1280GetRangingResult( RadioRangingResultTypes_t resultType );

/*!
 * \brief Sets the standard processing delay between Master and Slave
 *
 * \param [in]  cal           RxTx delay offset for correcting ranging bias.
 *
 * The calibration value reflects the group delay of the radio front end and
 * must be re-performed for each new SX1280 PCB design. The value is obtained
 * empirically by either conducted measurement in a known electrical length
 * coaxial RF cable (where the design is connectorised) or by radiated
 * measurement, at a known distance, where an antenna is present.
 * The result of the calibration process is that the SX1280 ranging result
 * accurately reflects the physical range, the calibration procedure therefore
 * removes the average timing error from the time-of-flight measurement for a
 * given design.
 *
 * The values are Spreading Factor dependents, and depend also of the board
 * design. Some typical values are provided in the next table.
 *
 * Spreading Factor | Calibration Value
 * ---------------- | -----------------
 *   SF5            |  12200
 *   SF6            |  12200
 *   SF7            |  12400
 *   SF8            |  12650
 *   SF9            |  12940
 *   SF10           |  13000
 *   SF11           |  13060
 *   SF12           |  13120
 */
void SX1280SetRangingCalibration( uint16_t cal );

/*!
 * \brief Clears the ranging filter
 */
void SX1280RangingClearFilterResult( void );

/*!
 * \brief Set the number of samples considered in the built-in filter
 *
 * \param [in]  numSample     The number of samples to use built-in filter
 *                            [8..255]
 *
 * \remark Value inferior to 8 will be silently set to 8
 */
void SX1280RangingSetFilterNumSamples( uint8_t numSample );


/*!
 * \brief Set the role of the radio during ranging operations
 *
 * \param [in]  role          Role of the radio
 */
void SX1280SetRangingRole( RadioRangingRoles_t role );

/*!
 * \brief Return the Estimated Frequency Error in LORA and RANGING operations
 *
 * \retval efe                The estimated frequency error [Hz]
 */
double SX1280GetFrequencyError( void );

/*!
 * \brief Returns the value of LoRa bandwidth from driver's value
 *
 * The value is returned in Hz so that it can be represented as an integer
 * type. Most computation should be done as integer to reduce floating
 * point related errors.
 *
 * \retval loRaBw              The value of the current bandwidth in Hz
 */
int32_t SX1280GetLoRaBandwidth( void );

/*!
 * \brief Set the driver in polling mode.
 *
 * In polling mode the application is responsible to call ProcessIrqs( ) to
 * execute callbacks functions.
 * The default mode is Interrupt Mode.
 * @code
 * // Initializations and callbacks declaration/definition
 * radio = SX1280( mosi, miso, sclk, nss, busy, int1, int2, int3, rst, &callbacks );
 * radio.Init( );
 * radio.SetPollingMode( );
 *
 * while( true )
 * {
 *                            //     IRQ processing is automatically done
 *     radio.ProcessIrqs( );  // <-- here, as well as callback functions
 *                            //     calls
 *     // Do some applicative work
 * }
 * @endcode
 *
 * \see SX1280SetInterruptMode
 */
void SX1280SetPollingMode( void );
/*!
 * \brief Set the driver in interrupt mode.
 *
 * In interrupt mode, the driver communicate with the radio during the
 * interruption by direct calls to ProcessIrqs( ). The main advantage is
 * the possibility to have low power application architecture.
 * This is the default mode.
 * @code
 * // Initializations and callbacks declaration/definition
 * radio = SX1280( mosi, miso, sclk, nss, busy, int1, int2, int3, rst, &callbacks );
 * radio.Init( );
 * radio.SetInterruptMode( );   // Optionnal. Driver default behavior
 *
 * while( true )
 * {
 *     // Do some applicative work
 * }
 * @endcode
 *
 * \see SX1280SetPollingMode
 */
void SX1280SetInterruptMode( void );

/*!
 * \brief DIOs interrupt callback
 *
 * \remark Called to handle all 3 DIOs pins
 */
void SX1280OnDioIrq( void );

/*!
 * \brief Process the analysis of radio IRQs and calls callback functions
 *        depending on radio state
 */
void SX1280ProcessIrqs( void );

#endif