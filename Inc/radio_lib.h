#ifndef RADIO_LIB_H
#define RADIO_LIB_H

#include <stdint.h>
#include <spi.h>

// It should be defined in project-name header not here in rfm73 library header
#define PACKET_MAX_BUFF_SIZE 30				// Bytes
#define PACKET_ADDRES_SIZE 5					// Bytes


// For debbug purposes
#ifdef __DBG_ITM
#define __DBG_SEND_CHAR(c) ITM_SendChar(c); ITM_SendChar('\n') //do {} while(0) //
#else
#define __DBG_SEND_CHAR(c) do {} while(0)
#endif





/** For future purposes ( if HAL will not be using anymore */
typedef SPI_HandleTypeDef R_SPI_HandleTypeDef ;	

/** If 8 bits would be not enough for Radio_TypeDef::status */
typedef uint8_t stat_TypeDef ;

/** 
*	Size of data in one packiet received from radio module ( max is 32 )
*/
#define R_BUFF_SIZE 32	






/** 
* \brief This structue contain whole necessary data in one place to manage RFM73 module,
*
*	Whole RFM73 library has been modified to use that structure. This allows you to
*	easily use more than one RFM73 module in project.
*	\pre To use RFM73 in project you need to fill this structure before you call
*	rfm73_init().
*
* 
* Data: 
* buffer[] table for incoming data from one packet ( up to 32B )
* buff_stat contain only status byte, read from radio module
* status only using by that library - it contain current state in 'state machine' ( bit description bellow ) 
* *spi_inst pointer to HAL stucture which is connected to certain instance SPI
* *A_SPI_gpio GPIO port for additional pins: CE, CSN, IRQ - per module
*/ 
struct Radio_TypeDef {
	
	/** Bytes just readed from RFM73 - data - not status register */
	volatile uint8_t buffer[R_BUFF_SIZE] ;			
	
	/** How many data bytes RFM73 has in RX FIFO. \n
	*	\note It also could be used in callback function to know how many bytes
	*	was received.
	*/
	volatile uint8_t buffer_maxl ;


	/** Used by non blocking procedure to read bytes from RFM73. \n
	*	
	*	\note It have not relevant information for programmer 
	*	\warning Do not save anything to this variable
	*/
	volatile uint8_t buffer_cpos ;
	
	/** Pointer to table which contain data to send.
	*	
	*	\note It is doesn't use in current version of library
	*/





	volatile uint8_t * volatile tx_buffer ;
	/** Size of table pointed by tx_buffer
	*
	*	\note It is doesn't use in current version of library
	*/
	volatile uint8_t tx_buff_size ;

	/** Point to position in tx_buffer when transmitting
	*	
	*	\note It is doesn't use in current version of library
	*/
	volatile uint8_t tx_buff_cpos ;							
	



	/** Pipe number - data in buffer[] come from that pipe */
	volatile uint8_t pipe ;
	/** RFM73's status register */
	volatile uint8_t buff_stat ;

	/** Status register for RFM73 library's functions
	*	\sa To get meaning bits this variable you need to see: RFM73_IRQ_OCR_BIT,
	*	RFM73_D_READING_BIT, RFM73_D_READY_BIT, RFM73_D_PENDING_BIT, RFM73_D_PENDING_BIT, 
	* 	RFM73_SPI_SENDING_BIT
	*/
	volatile stat_TypeDef status ;
	

	/** Pointer to user callback functions to handle event:
	* 	\li max. retransmissions - receiver didn't get packet
	*	
	*	\param struct Radio_TypeDef * poiner to structure which describe RFM73
	*	\return return \b non-zero value to FLUSH TX buffer
	*/
	uint8_t (*_max_retransmission_handler)( struct Radio_TypeDef * ) ;


	/** Pointer to user callback functions to handle event:
	* 	\li packet was send properly - receiver got packet
	*	
	*	\param struct Radio_TypeDef * poiner to structure which describe RFM73
	*/
	void (*_packet_sent_handler)( struct Radio_TypeDef * ) ;

	

	/** Pointer to user callback functions to handle event:
	* 	\li data ready to read
	*	
	*	\param struct Radio_TypeDef * poiner to structure which describe RFM73
	*/
	void (*_data_ready_handler)( struct Radio_TypeDef *) ;
	



	/** Pointer to STM HAL SPI structure
	*	\note RFM73 libary use only \c Instance element from STM HAL SPI structure, so
	*	if you do not want to use STM HAL drivers in project you need to create
	*	structure which will contain one element: SPI;s \c Instance and assing
	*	it address to this pointer.
	*/
	R_SPI_HandleTypeDef *spi_inst ;
	/** SPI IRQ number */
	IRQn_Type spi_irqn ;

	/**  PORT ( CMSIS structure ) where Additional pins CE and CSN are connected */
	GPIO_TypeDef *A_SPI_gpio_port ;

	/** CE pin mask */
	uint16_t A_SPI_CE_pin ;
	/** CSN pin mask */
	uint16_t A_SPI_CSN_pin ;
	
	/** PORT (CMSIS structure) where IRQ pin is conected */
	GPIO_TypeDef *A_IRQ_gpio_port ;
	/** IRQ pin mask */
	uint16_t A_SPI_IRQ_pin ;										
	
} ;


/** \brief Desribes bit 0 meaning in Radio_TypeDef::status
*
*	Is set by IRQ falling interrupt - inform main that RFM73 module
*	has someting to say
*/
#define RFM73_IRQ_OCR_BIT 0
#define RFM73_IRQ_OCR_MASK (stat_TypeDef)( 1 << RFM73_IRQ_OCR_BIT )

/** \brief Desribes bit 1 meaning in Radio_TypeDef::status
*
*	If is set indicates that data are reading form RFM73 module
*/
#define RFM73_D_READING_BIT 1
#define RFM73_D_READING_MASK (stat_TypeDef)( 1 << RFM73_D_READING_BIT )

/**	\brief Desribes bit 2 meaning in Radio_TypeDef::status
*
*	Is set when all bytes from RFM73 module has been read
*/
#define RFM73_D_READY_BIT 2
#define RFM73_D_READY_MASK (stat_TypeDef)( 1 << RFM73_D_READY_BIT )


#define RFM73_D_PENDING_BIT 3
#define RFM73_D_PENDING_MASK (stat_TypeDef)( 1 << RFM73_D_PENDING_BIT )


/** \brief Desribes bit 4 meaning in Radio_TypeDef::status
*
*	Is set when byte is reading from RFM73 and other bytes are waiting to read
*/
#define RFM73_SPI_SENDING_BIT 4
#define RFM73_SPI_SENDING_MASK (stat_TypeDef)( 1 << RFM73_SPI_SENDING_BIT )




//***************************************************************************//
//
// COPYRIGHT NOTICE (zlib license)
//
// Loosely based on the example application provided by HopeRF
//
// (c) Wouter van Ooijen - wouter@voti.nl
//
//  This software is provided 'as-is', without any express or implied
//  warranty.  In no event will the authors be held liable for any damages
//  arising from the use of this software.
//
//  Permission is granted to anyone to use this software for any purpose,
//  including commercial applications, and to alter it and redistribute it
//  freely, subject to the following restrictions:
//
//  1. The origin of this software must not be misrepresented; you must not
//     claim that you wrote the original software. If you use this software
//     in a product, an acknowledgment in the product documentation would be
//     appreciated but is not required.
//  2. Altered source versions must be plainly marked as such, and must not be
//     misrepresented as being the original software.
//  3. This notice may not be removed or altered from any source distribution.
//
//***************************************************************************//

//***************************************************************************//
//
//! \defgroup lowlevel low level interface
//!
//! The low-level interface provides read and write access to the 
//! commands and registers of the RFM73.
//!
//! When a register ( < 0x20 ) is specified for a read or write command
//! the appropriate read or write command bits are added automatically.
//!
//! To use any of these functions, (except rfm73_init() itself)
//! the interface and the module must have been initialized by
//! an rfm73_init() call.
//!
//! Besides the registers shown here (bank 0) the rfm73 also has a
//! set of registers in bank 1. These bank 1 registers are initialized by
//! the rfm73_init() call. If you want to do this yourself: the datasheet
//! shows the required values, but in a very confusing way. The HopeRF 
//! example code is a better reference. No (or very scarce) explanation 
//! is given for these values.
//!
//! For most users, especially novices, it is recommended to use the
//! high level interaface instead of this low level interface.
//
//***************************************************************************//

//***************************************************************************//
//
//! \defgroup highlevel high level interface
//!
//! The high-level interface provides functions for using the rfm73 module.
//! These functions are implemneted by calling the appropriate low level
//! functions. 
//! When possible, it is recommended to use only these high level functions.
//! But when a functionality is needed that is missing it can be implemented
//! using the low level interface.
//!
//! To use any of these functions, (except rfm73_init() itself)
//! the interface and the module must have been initialized by
//! an rfm73_init() call.
//
//***************************************************************************//


//! version of this rfm73 library
//
//! \ingroup highlevel
#define RFM73_LIB_VERSION "V1.04 (2013-06-14)"

//! maximum number of data bytes in a (received or transmitted) rfm73 packet
//
//! \ingroup highlevel
#define RFM73_MAX_PACKET_LEN  32

//! type of rfm73 (transmit or receive) buffer
//
//! \ingroup highlevel
typedef unsigned char rfm73_buffer [ RFM73_MAX_PACKET_LEN ];


//***************************************************************************//
//
// RFM73 SPI commands
//
//***************************************************************************//

//! \addtogroup lowlevel
//! @{

//! SPI comamnd to read a received payload
#define RFM73_CMD_R_RX_PAYLOAD         0x61

//! SPI command to write a payload to be sent
#define RFM73_CMD_W_TX_PAYLOAD         0xA0

//! SPI command to empty the transmit queue
#define RFM73_CMD_FLUSH_TX             0xE1

//! SPI command to empty the receive queue
#define RFM73_CMD_FLUSH_RX             0xE2

//! SPI command to start continuous retransmission
#define RFM73_CMD_REUSE_TX_PL          0xE3

//! SPI command to write a payload to be sent without auto-acknowledgement
#define RFM73_CMD_W_TX_PAYLOAD_NOACK   0xB0

//! SPI command to write the payload to be transmitted with an ack
#define RFM73_CMD_W_ACK_PAYLOAD        0xA8

//! SPI command to toggle register bank or toggle extended functions
#define RFM73_CMD_ACTIVATE             0x50

//! SPI command to read the payload length for the top payload in the FIFO
#define RFM73_CMD_R_RX_PL_WID          0x60

//! SPI 'no peration', can be used to read the status register
#define RFM73_CMD_NOP                  0xFF

//***************************************************************************//
//
// RFM73 register addresses
//
//***************************************************************************//

//! CONFIG : rfm73 configuration register
//
//! Bits (0 = LSB):
//! - 7 : reserved, must be 0
//! - 6 : 1 masks RX_DR (see...) from IRQ pin, 0 allows
//! - 5 : 1 masks RX_DS (see...) from IRQ pin, 0 allows
//! - 4 : 1 masks MAX_RT (see...) from IRQ pin, 0 allows
//! - 3 : 1 enables CRC (forced high when EN_AA != 0)
//! - 2 : 0 = 1 byte CRC, 1 = 2 byte CRC
//! - 1 : 0 = power down, 1 = power up
//! - 0 : 0 = transmit mode, 1 = receive mode
#define RFM73_REG_CONFIG               0x00

//! EN_AA : enable auto ack on pipes
//
//! Bits (0 = LSB):
//! - 7, 6 : reserved, must be 00
//! - 5 : 0 disables auto ack on pipe 5, 1 enables
//! - 4 : 0 disables auto ack on pipe 4, 1 enables
//! - 3 : 0 disables auto ack on pipe 3, 1 enables
//! - 2 : 0 disables auto ack on pipe 2, 1 enables
//! - 1 : 0 disables auto ack on pipe 1, 1 enables
//! - 0 : 0 disables auto ack on pipe 0, 1 enables
#define RFM73_REG_EN_AA                0x01

//! EN_RXADDR : enable receive pipes
//
//! Bits (0 = LSB):
//! - 7, 6 : reserved, must be 00
//! - 5 : 0 disables receive pipe 5, 1 enables
//! - 4 : 0 disables receive pipe 4, 1 enables
//! - 3 : 0 disables receive pipe 3, 1 enables
//! - 2 : 0 disables receive pipe 2, 1 enables
//! - 1 : 0 disables receive pipe 1, 1 enables
//! - 0 : 0 disables receive pipe 0, 1 enables
#define RFM73_REG_EN_RXADDR            0x02

//! SETUP_AW : set address length
//
//! Bits (0 = LSB):
//! - 7 .. 2 : reserved, must be 000000
//! - 1 .. 0 : 00 = illegal, 01 = 3 bytes, 10 = 4 bytes, 11 = 5 bytes
#define RFM73_REG_SETUP_AW             0x03

//! SETUP_RETR : retransmission settings
//
//! Bits (0 = LSB):
//! - 7 .. 4 : delay between (re) transmissions, ( n + 1 ) * 250 us
//! - 3 .. 0 : max number of retransmissions, 0 disableles retransmissions
#define RFM73_REG_SETUP_RETR           0x04

//! RF_CH : RF channel (frequency)
//
//! The RF channel frequency is 2.4 MHz + n * 1 MHz.
#define RFM73_REG_RF_CH                0x05

//! RF_SETUP : RF setup: data rate, transmit power, LNA
//
//! Bits (0 = LSB):
//! - 7 .. 6 : reserved, must be 00
//! - 5 : air data rate low bit
//! - 4 : reserved, must be 0
//! - 3 : air data rate high bit, 00 = 1 Mbps, 01 = 2 Mbps, 10 = 250Kbps
//! - 2 .. 1 : transmit power, 00 = -10 dBm, 01 = -5 dBm, 10 = 0 dBm, 11 = 5 dBm
//! - 0 : LNA gain, 0 = - 20 dB (low gain), 1 = standard
#define RFM73_REG_RF_SETUP             0x06

//! STATUS : status register
//
//! The value of this register is also clocked out
//! while a SPI command is clocked in.
//!
//! Bits (0 = LSB):
//! - 7 : active register bank, 0 = bank 0, 1 = bank 1
//! - 6 : data available, 0 = RX FIFO not empty, 1 = RX FIFO empty
//! - 5 : data sent, 0 = no packet sent, 1 = packet has been sent
//! - 4 : 1 = maximum number of retransmissions reached
//! - 3 .. 1 : data pipe of the message at the RX queue head
//! - 0 : TX FIFO full: 0 = TX FIFO not full, 1 = TX FIFO full
//!
//! Bits 6,5,4 are cleared by writing a 1 (!) in that position.
//! When bit 4 is set this will block any communication.
//! When auto retransmission is enabled bit 5 will be set only
//! after the acknowledge has been received.
#define RFM73_REG_STATUS               0x07

//! OBSERVE_TX : lost and retransmitted packets
//
//! Bits (0 = LSB):
//! - 7 .. 4 : counts number of lost packets
//! - 3 .. 0 : counts retranmits 
//! The lost packets counter will not increment beyond 15. 
//! It is reset by writing to the channel frequency register.
//!
//! The retransmits counter can not increment beyond 15 because
//! the maximum number of transmissions is 15. This counter
//! is reset when the transmission of a new packet starts.
#define RFM73_REG_OBSERVE_TX           0x08

//! CD : carrier detect
//
//! Bits (0 = LSB):
//! - 7 .. 1 : reserved
//! - 1 : carrier detect
#define RFM73_REG_CD                   0x09

//! RX_ADDR_PO : receive address for data pipe 0, 5 bytes
//
//! This is the (up to) 5 byte receive address for data pipe 0.
//! For auto acknowledgement to work this address must be 
//! the same as the transmit address.
#define RFM73_REG_RX_ADDR_P0           0x0A

//! RX_ADDR_P1 : receive address for data pipe 1, 5 bytes
//
//! This is the (up to) 5 byte receive address for data pipe 1.
//! The higher bytes (all but the LSB) are also used in
//! the receive addresses of data pipes 2 .. 5.
#define RFM73_REG_RX_ADDR_P1           0x0B

//! RX_ADDR_P2 : receive address for data pipe 2, 1 byte
//
//! This is the LSB of the receive address for data pipe 2.
//! The higher bytes are copied from the receive address of
//! data pipe 1.
#define RFM73_REG_RX_ADDR_P2           0x0C

//! RX_ADDR_P3 : receive address for data pipe 3, 1 byte
//
//! This is the LSB of the receive address for data pipe 3.
//! The higher bytes are copied from the receive address of
//! data pipe 1.
#define RFM73_REG_RX_ADDR_P3           0x0D

//! RX_ADDR_P4 : receive address for data pipe 4, 1 byte
//
//! This is the LSB of the receive address for data pipe 4.
//! The higher bytes are copied from the receive address of
//! data pipe 1.
#define RFM73_REG_RX_ADDR_P4           0x0E

//! RX_ADDR_P5 : receive address for data pipe 5, 1 byte
//
//! This is the LSB of the receive address for data pipe 2.
//! The higher bytes are copied from the receive address of
//! data pipe 5.
#define RFM73_REG_RX_ADDR_P5           0x0F

//! TX_ADDR : tranmsit adress, 5 bytes
//
//! This is the (up to) 5 byte adress used in transmitted packets.
//! For auto acknowledgement to work this address must be 
//! the same as the pipe 0 receive address.
#define RFM73_REG_TX_ADDR              0x10

//! RX_PW_P0 : number of bytes in package received into pipe 0
//
//! This is the number of data bytes in the message at
//! the head of receive pipe 0.
#define RFM73_REG_RX_PW_P0             0x11

//! RX_PW_P1 : number of bytes in package received into pipe 1
//
//! This is the number of data bytes in the message at
//! the head of receive pipe 1.
#define RFM73_REG_RX_PW_P1             0x12

//! RX_PW_P2 : number of bytes in package received into pipe 2
//
//! This is the number of data bytes in the message at
//! the head of receive pipe 2.
#define RFM73_REG_RX_PW_P2             0x13

//! RX_PW_P3 : number of bytes in package received into pipe 3
//
//! This is the number of data bytes in the message at
//! the head of receive pipe 3.
#define RFM73_REG_RX_PW_P3             0x14

//! RX_PW_P4 : number of bytes in package received into pipe 4
//
//! This is the number of data bytes in the message at
//! the head of receive pipe 4.
#define RFM73_REG_RX_PW_P4             0x15

//! RX_PW_P5 : number of bytes in package received into pipe 5
//
//! This is the number of data bytes in the message at
//! the head of receive pipe 5.
#define RFM73_REG_RX_PW_P5             0x16

//! FIFO_STATUS : receive and transmit FIFO status (readonly)
//
//! Bits (0 = LSB):
//! - 7   : reserved, only 0 allowed
//! - 6   : high = re-use last transmitted packet
//! - 5   : high = transmit FIFO is full
//! - 4   : high = transmit FIFO is empty
//! - 3:2 : reserved, only 0 allowed
//! - 1   : high = receive FIFO is full
//! - 0   : high = receive FIFO is empty
#define RFM73_REG_FIFO_STATUS          0x17

//! DYNPD: dynamic payload flags
//
//! Bits (0 = LSB):
//! - 7:6 : reserved, only 00 allowed
//! - 5   : high = dynamic payload enabled on data pipe 5
//! - 4   : high = dynamic payload enabled on data pipe 4
//! - 3   : high = dynamic payload enabled on data pipe 3
//! - 2   : high = dynamic payload enabled on data pipe 2
//! - 1   : high = dynamic payload enabled on data pipe 1
//! - 0   : high = dynamic payload enabled on data pipe 0
//! Setting dynamic payload on pipe x requires EN_DPL 
//! (in the special features flags register) and ENAA_Px.
#define RFM73_REG_DYNPD                0x1C

//! FEATURE: special fature flags
//
//! Bits (0 = LSB):
//! - 7:3 : reserved, only 00000 allowed
//! - 2   : (EN_DPL) high = enable dynamic payload length
//! - 1   : (EN_ACK_PAY) high = enable payload with ack
//! - 0   : (EN_DYN_ACK) high = enables W_TX_PAYLOAD_NOACK command 
#define RFM73_REG_FEATURE              0x1D

//! @}




//! initialize the library and the rfm73 module
//
//! \ingroup lowlevel
//! This function must be called before any other rfm73 
//! function is called. It can also be called later (maybe even
//! periodically) to re-initialize the interafce and the module.
//!
//! The rfm73 is initialized to
//! - 2 byte CRC
//! - power up
//! - receive mode
//! - auto-acknowledge on all pipes enabled
//! - pipes 0 and 1 are enabled, others disabled
//! - use 5 byte addresses
//! - auto retransmission delay 4000 ms, retry 15 times
//! - use channel 10
//! - air data rate 1Mbit power 5dbm, LNA gain high
//! - use some fixed address
void rfm73_init( struct Radio_TypeDef * );

//! read a single-byte command or register
//
//! \ingroup lowlevel
//! This function reads and returns the a single-byte (8 bit) 
//! RFM73 command or register reg. 
unsigned char rfm73_register_read( struct Radio_TypeDef * _radioH, unsigned char reg );

//! read a multi-byte command or register
//
//! \ingroup lowlevel
//! This function reads length bytes (8 bit each) from the RFM73 
//! command or register reg into the buffer buf. 
void rfm73_buffer_read(
	 struct Radio_TypeDef *,
   unsigned char reg,
   unsigned char buf[],
   unsigned char length
);

//! write a single-byte command or register
//
//! \ingroup lowlevel
//! This function writes the single-byte (8 bit) val to
//! the  RFM73 command or register reg. 
void rfm73_register_write( struct Radio_TypeDef * _radioH, unsigned char reg, unsigned char val );

//! write a multi-byte command or register
//
//! \ingroup lowlevel
//! This function writes length bytes (8 bit each) from 
//! the buffer buf into the RFM73 
//! command or register reg. 
void rfm73_buffer_write(
	 struct Radio_TypeDef *,
   char reg,
   const unsigned char buf[],
   unsigned char length
);

//***************************************************************************//
//
// high-level interface
//
//***************************************************************************//

//! report whether the rfm73 module is present
//
//! \ingroup highlevel
//! This function tests whether the rfm73 module is present.
//! It does so by reading the status register, and verifying that
//! an activate 0x53 command toggles the RBANK bit
//! in the status register.
unsigned char rfm73_is_present( struct Radio_TypeDef * );

//! switch the rfm73 to transmit mode
//
//! \ingroup highlevel
//! This function flushes the transmit queue,
//! and switches the rfm73 to transmit mode.
void rfm73_mode_transmit( struct Radio_TypeDef * );

//! switch the rfm73 to receive mode
//
//! \ingroup highlevel
//! This function flushes the receive queue,
//! and switches the rfm73 to transmit mode.
void rfm73_mode_receive( struct Radio_TypeDef * );

//! switch the rfm73 to standby mode
//
//! \ingroup highlevel
//! This function puts the rfm73 in standby I mode,
//! which reduces the power consumption 
//! (50 uA max).
//! rfm73_mode_powerdown() reduces the power consumption
//! even further, but requires a longer (but unspecified?)
//! time to return to active mode.
void rfm73_mode_standby( struct Radio_TypeDef * );

//! switch the rfm73 to power down mode
//
//! \ingroup highlevel
//! This function puts the rfm73 in power down mode,
//! which reduces the power consumption to a minimum
//! ( 3 uA max). 
void rfm73_mode_powerdown( struct Radio_TypeDef * );

//! set the rfm73 lna gain to low
//
//! \ingroup highlevel
//! This sets the LNA gain of the receiver to the low
//! level (-20 dB compared to the 'high' level).
void rfm73_lna_low( struct Radio_TypeDef * );

//! set the rfm73 lna gain to high
//
//! \ingroup highlevel
//! This sets the LNA gain of the receiver to the so-called
//! 'high' level. (I would have called it the 'normal' level.)
void rfm73_lna_high( struct Radio_TypeDef * );

//! set the rfm73 channel frequency
//
//! \ingroup highlevel
//! This function sets the frequency (channel) used by the rfm73 for 
//! receiving and transmitting to ( 2400 + ch ) MHz.
//! The highest bit of val is ignored, so the frequency range is
//! 2.4 .. 2.517 GHz. 
//! Not all of these frequencies might be free to use in your jurisdiction.
void rfm73_channel( struct Radio_TypeDef *, unsigned char ch );

//! set the rfm73 air data rate (baudrate)
//
//! \ingroup highlevel
//! This function sets the air data rate used by the rfm73 for 
//! receiving and transmitting. 
//! Allowed values are 0 (250 Kbps), 1 (1 MBps) and 2 (Mbps).
//! A value > 2 will have the same effect as a value of 2.
//! Note that this is the bitrate the rfm73 uses in the 
//! packages that it sends. Due to various overhead factors
//! the data rate that a user of the module can achieve is much lower,
//! probably by a factor of 4.
void rfm73_air_data_rate( struct Radio_TypeDef *, unsigned char rate );

//! set the rfm73 CRC length
//
//! \ingroup highlevel
//! This function sets the length of the CRC used by the rfm73 in bytes.
//! Valid values are 0 (no CRC), 1 and 2.
//! A value > 2 has the same effect as the value 2.
//! Disabling the CRC disables the auto-acknowledge function.
//! Enabling the CRC does not automatically enable the
//! auto-acknowledge function.
void rfm73_crc_length( struct Radio_TypeDef *, unsigned char len );

//! set the rfm73 address length
//
//! \ingroup highlevel
//! This function sets the length (in bytes) of the addresses used by 
//! the rfm73. Valid values are 3, 4 and 5.
//! A value < 3 has the same effect as the value 3.
//! A value > 5 has the same effect as the value 5.
//! This setting is common for all data pipes.
void rfm73_address_length( struct Radio_TypeDef *, unsigned char len );

//! set the rfm73 transmit power
//
//! \ingroup highlevel
//! This function sets the rfm73 transmitter output power level.
//! - level == 0 => -10 dBm
//! - level == 1 =>  -5 dBm
//! - level == 2 =>   0 dBm
//! - level == 3 =>  +5 dBm
//! 
//! A level > 3 has the same effect as level == 3.
void rfm73_power( struct Radio_TypeDef *, unsigned char level ); 

//! set the retransmission delay and number of attempts
//
//! \ingroup highlevel
//! This function sets the delay d between retransmission attempts,
//! and the maximum number of attempts n.
//! The range of both arguments is 0..15. 
//! A value > 15 has the same effect as the value 15.
//!
//! The retransmission delay d is specified in steps of 250 us
//! with a minimum of 250 us:
//! - d == 0 => 250 us
//! - d == 1 => 500 us
//! - ...
//! - d == 15 => 4 ms
//!
//! The number of retransmissions n can range from
//! 0 (effectively disabling auto-retransmission) to 15.
void rfm73_retransmit_delay_attempts(
		struct Radio_TypeDef *,
		unsigned char d,
		unsigned char n
		);

//! read rfm73 retransmit count
//
//! \ingroup highlevel
//! This function reads and reports the number of retransmissions 
//! for the last packet that was sent. The number of retransmissions
//! is reset to zero when a new packet is sent.
unsigned char rfm73_retransmit_count( struct Radio_TypeDef * );

//! read rfm73 lost packets count
//
//! \ingroup highlevel
//! This function reads and reports the number of lost packets.
//! The range of this count is 0..15, at 15 it will not
//! increment when a next packet is lost.
//! The lost packets count is reset implicitly when the channel
//! is written (by calling rfm73_channel() ) or 
//! explicitly by calling rfm73_lost_packets_reset().
unsigned char rfm73_lost_packets_count( struct Radio_TypeDef * );

//! reset rfm73 lost packets count
//
//! \ingroup highlevel
//! This function resets the number of lost packets by reading and
//! re-writing the RF channel (RFM73_REG_RF_CH register).
void rfm73_lost_packets_reset( struct Radio_TypeDef * );

//! enables or disables the autoack on a pipe
//
//! \ingroup highlevel
//! This function enables or disables the auto acknowledgement
//! function on the specified pipe.
//!
//! pipe must be in the range 0..5. 
//! A pipe > 5 has the same effect as using pipe 5. 
void rfm73_pipe_autoack(
		struct Radio_TypeDef *,
		unsigned char pipe,
		unsigned char enabled
		);

//! enables or disables a pipe
//
//! \ingroup highlevel
//! This function enables or disables the specified pipe.
//!
//! pipe must be in the range 0..5. 
//! A pipe > 5 has the same effect as using pipe 5. 
void rfm73_pipe_enable(
		struct Radio_TypeDef *,
		unsigned char d,
		unsigned char enabled
		);

//! set the rfm73 pipe 0 address
//
//! \ingroup highlevel
//! This function sets the (up to 5 byte) receive address of pipe 0.
//! When the address length is set to less than 5 the excess
//! bytes are ignored, but address must still be a 5 byte array.
//!
//! Pipes 0 and 1 have a full (up to 5) byte address.
//! The other pipes (2..5) copy all but the least significant
//! (= first) byte from the pipe 1 address.
void rfm73_receive_address_p0( 
	struct Radio_TypeDef *, 
	const unsigned char address[ 5 ]
	);

//! set the rfm73 pipe 1 address
//
//! \ingroup highlevel
//! This function sets the (up to 5 byte) receive address of pipe 1.
//! When the address length is set to less than 5 the excess
//! bytes are ignored, but address must still be a 5 byte array.
//!
//! Pipes 0 and 1 have a full (up to 5) byte address.
//! The other pipes (2..5) copy all but the least significant
//! (= first) byte from the pipe 1 address.
void rfm73_receive_address_p1( 
	struct Radio_TypeDef *,
	const unsigned char address[ 5 ] 
	);

//! set the rfm73 pipe n (2..5) address
//
//! \ingroup highlevel
//! This function sets the least significant byte of 
//! the receive address of the pipe n.
//! The other bytes of the address are copied from pipe 1.
void rfm73_receive_address_pn( 
	struct Radio_TypeDef *,
	unsigned char channel,
	unsigned char address 
	);

//! set the payload size for pipe n
//
//! \ingroup highlevel
//! This function sets the size (= number of bytes, can be 1..32) 
//! for packets to be received  on pipe n. 
//! This setting must be the same as on the tranmitter.
//! A size of 0 will enable dynamic length packets.
//! A size > 32 will have the same effect as a size of 32.
void rfm73_channel_payload_size( 
	struct Radio_TypeDef *,
	unsigned char n,
	unsigned char size 
	);

//! set the rfm73 transmit address
//
//! \ingroup highlevel
//! This function sets the (up to 5 byte) address used
//! for all transmissions.
void rfm73_transmit_address( 
	struct Radio_TypeDef *,
	const unsigned char address[] 
	);   

//! report whether the transmit fifo is full
//
//! \ingroup highlevel
//! This function reads and reports whether the transmit fifo is full.
unsigned char rfm73_transmit_fifo_full( struct Radio_TypeDef * );

//! report whether the receive fifo is empty
//
//! \ingroup highlevel
//! This function reads and reports whether the receive fifo is full.
unsigned char rfm73_receive_fifo_empty( struct Radio_TypeDef * );

//! transmit a message
//
//! \ingroup highlevel
//! This function transmits the specified message.
//!
//! The specified length must be less than or equal to 
//! RFM73_MAX_PACKET_LEN (32).
//! Specifying a larger length has the same effect as
//! specifying a length of RFM73_MAX_PACKET_LEN.
//!
//! The retransmission setting (set by
//! the function rfm73_retransmit_delay_attempts) determines
//! whether the message is transmitted on the air just once
//! or repeatedly until an acknowledge is received. 
//! 
//! The RFM73 must be in transmit mode.
void rfm73_transmit_message(
	 struct Radio_TypeDef * _radioH,
   const unsigned char buf[],
   unsigned char length
);

//! transmit a message once
//
//! \ingroup highlevel
//! This function transmits the specified message once.
//!
//! The specified length must be less than or equal to 
//! RFM73_MAX_PACKET_LEN (32).
//! Specifying a larger length has the same effect as
//! specifying a length of RFM73_MAX_PACKET_LEN.
//!
//! The message is transmitted on the air once, irrespective
//! of the retransmission setting. 
//! 
//! The RFM73 must be in transmit mode.
void rfm73_transmit_message_once(
	 struct Radio_TypeDef * _radioH,
   const unsigned char buf[],
   unsigned char length
);

//! get pipe number of the next message in receive FIFO
//
//! \ingroup highlevel
//! This function returns the number of the
//! pipe (0..5) on which the head message in the receive FIFO
//! was received.
//!
//! When the receive fifo is empty 0x07 is returned.
unsigned char rfm73_receive_next_pipe( struct Radio_TypeDef * );

//! get payload length of the next message in receive FIFO
//
//! \ingroup highlevel
//! This function returns length of the head message 
//! in the receive FIFO in bytes (1..32).
//! 
//! The RFM73 datasheet does not specify the value that is 
//! returned when the receive FIFO is empty
unsigned char rfm73_receive_next_length( struct Radio_TypeDef * );

//! (try to) receive a message
//
//! \ingroup highlevel
//! This function tries to receive a message.
//!
//! This function tries to retrieve a received message
//! from the receive FIFO. When no message is available
//! this function returns false. When a message is avaible
//! it is retrieved. The data is put in the buffer buf,
//! the length is written to length, and the function 
//! returns true.
//!
//! The size of the buffer buf must be at least
//! RFM73_MAX_PACKET_LEN (32).
//! 
//! The RFM73 must be in transmit mode.
unsigned char rfm73_receive(
	 struct Radio_TypeDef *,
   unsigned char * pipe,
   unsigned char buf[],
   unsigned char * length
);


	 
/*
	Additional functions
*/	 
	 
/**
	* @brief Funtion which perform SPI transmiting
	*
*/
uint8_t rfm73_SPI_RW (	R_SPI_HandleTypeDef *_spiH, uint8_t _data ) ;

void rfm73_bank( struct Radio_TypeDef * _radioH, unsigned char b ) ;
void rfm73_init_bank1( struct Radio_TypeDef * ) ;
void rfm73_wait_ms( uint32_t );

void RFM73_CSN (struct Radio_TypeDef * , uint8_t ) ;
void RFM73_CE( struct Radio_TypeDef * , uint8_t  ) ;

/**
*	\brief	Analzying request from RFM73 when IRQ interrupt sets RFM73_IRQ_OCR bit
*
*/
void _rfm73_analyze( struct Radio_TypeDef * ) ;

/**
*	\note That functions need to be called form main - it stearing whole 
* 	data flow from RFM73 to MCU
*/
void rfm73_check( struct Radio_TypeDef * ) ;

/** 
*	\brief That function must be called in SPI interrupt - when RX data must be handled
* 	
* 	All bytes will be save to buffer in Radio_TypeDef  structure in non bocking way
*/
void rfm73_rx_interrupt_handle ( struct Radio_TypeDef * ) ;



/**
*	\brief It is using for setting transmission pipe - used by transmitter
* 	
* 	Call this function once before sending data.\n
*	It sets tranmitter address in RFM73 module and address for pipe 0 for
*	auto-acknowledge function.
* 	\n\n You need to also call rfm73_mode_transmit() once before sending data.
*/
void rfm73_set_Tpipe (
	struct Radio_TypeDef * _radioH, 
	const uint8_t * const _pipe_addr
	) ;


/** 
*   DOES NOT WORK PROPERLY FOR NOW
*		@brief That function must be called in SPI interrupt - when TX data must be handled
* 	
* 	This function is connected with rfm73_init_sendingNB()
*/
//void rfm73_tx_interrupt_handle ( struct Radio_TypeDef * ) ;




/**
* 	DOES NOT WORK PROPERLY FOR NOW
* 	@brief Initialize non blocking sending data by SPI to RFM73 module
* 	
*/


/*
uint8_t rfm73_init_sendingNB(
	struct Radio_TypeDef * _radioH, 
	uint8_t * const _buff,
	uint8_t _buff_size,
	const uint8_t * const _pipe_addr
);
*/

#endif
