
//**************************************************************************
//
//! \mainpage rfm73
//!
//! C interface library for the HopeRF RFM73 2.4 GHz transceiver module
//!
//! \image html RFM73-D.jpg
//! \image rtf RFM73-D.jpg
//!
//! \author 
//!    Wouter van Ooijen (wouter@voti.nl)
//!
//! \version
//!    V1.04 (2013-06-14)
//!
//! \par Introduction
//!
//! The RFM73 module from HopeRF (www.hoperf.com) is a small and cheap
//! transceiver (transmitter + receiver) module for the license-free
//! 2.4 MHz frequency band. This module is intended for short range
//! communication, for instance within a room. 
//! Typical applications are wires mouses and keybords, 
//! and garage door openers.
//! In open air a distance of 
//! 50 .. 100 m is possible, but this is strictly line-of-sight: even 
//! the leaves of a single tree can obstruct the signal.
//!
//! The two main files in this libary, rfm73.h and rfm73.c,
//! are target independent. 
//! However, rfm73.c included by a file that must be provided
//! by the user. 
//! This file must provide macro's for initializing and accessing the
//! I/O pins that connect to the RFM73 module, and for
//! delays of a specified number of milliseconds and microseconds.
//! A template for this file is provided.
//!
//! \par Files
//! - rfm73.h
//! - rfm73.c
//! - rfm73-config.h
//!
//!
//! \par License
//!
//! The files in this library are licensed under the zlib license, 
//! which rougly says: when you redistribute the *source* you must 
//! keep the copyright notice intact. Otherwise you can use these files
//! any way you like, including getting rich from selling applications.
//! But don't complain to me when it does not work (OTOH, contructive
//! comments are appreciated!).
//!
//! \par ToDo: bugs, notes, pitfalls, todo, known problems, etc
//!
//! - pictures for power
//! - document (other) files
//! - PIC18F examples, Cortex C examples, Arduino
//! - write 5 bytes when address <5 bytes?
//! - check and improve rtf documentation
//! - check CE cycling requirements
//! - descriptions for the other files
//! - add write ack payload for pipe N
//! - add 'report package has been sent'
//! - application: thermometer
//! - application: test signal strength
//! - application: asynch serial through the air
//!
//
//***************************************************************************//

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
//! \page revision information
//!
//! \par ToDo: bugs, notes, pitfalls, todo, known problems, etc
//!
//! - pictures for power
//! - document (other) files
//! - PIC18F examples, Cortex C examples, Arduino
//! - write 5 bytes when address <5 bytes?
//! - check and improve rtf documentation
//! - check CE cycling requirements
//! - descriptions for the other files
//! - add write ack payload for pipe N
//! - add 'report package has been sent'
//! - application: thermometer
//! - application: test signal strength
//! - application: asynch serial through the air
//! 
//! \par release notes
//!
//! V1.03 (2012-11-16)
//! Copied from the RFM70 version and adapted
//! 
//
//***************************************************************************//

//***************************************************************************//
//
//! \page overview RFM73 overview
//!
//! \image html smd-pins.jpg
//! \image rtf smd-pins.jpg
//!
//! The RFM73 is a small module (PCB with some components, ~ 18 x 26 mm) 
//! that implements a transciever for the license-free 2.4 GHz band. 
//! The module contains a small antenna on the PCB, 
//! there is no provision for an external antenna.
//! Two versions of the module exist, one with 
//! pins and one without. The pins are on a 1.28 mm grid, which is twice the
//! density of a standard DIL package. The module requires 1.9 .. 3.6 Volt
//! for its power, 3.3 Volt seems to be the preferred operating voltage. 
//! It is NOT designed to run at 5.0 Volt. 
//! According to the datasheet that maximum 'typical' operating current 
//! is 23 mA.
//!
//! Note that 'licence free' means that certain well-described use of this 
//! frequency band is allowd without a license. 
//! The RFM73 is designed to be compatible with the requirements for such 
//! use, but it is the responsibility of anyone who sells a product that
//! to make sure that it complies with such rules.
//!
//! The main component of an RFM73 module is the RF70 chip (hidden beneath
//! the black blob). The manufacturer provides separate datasheets for both 
//! the RF70 chip and the RFM73 module. 
//! The two datasheets seem to be the same, except for the physical 
//! and pinout information which is of course different for the chip
//! and the module, so you can probably ignore the RF70 chip datasheet.
//! The RFM73 module datasheet can currently be found at 
//! http://www.hoperf.com/upload/rf/RFM73.pdf
//!
//! The RFM73 module is intended for short-range communication,
//! like wireless computer peripherals (mouse, keyboard, tablet, ect.)
//! keyfobs (car opener, garage door opener, motorized fence opener - 
//! some cryptografy will probably be required for such applications)
//! and toys. In a line of sight situation a maximum range of 50 .. 100 m
//! is possible. Indoors communication within a single room will generally
//! be OK (unless you have a very large room..) but passing even a single
//! wall can be a problem.
//!
//! An RFM73 module must be used with a microcontroller that initializes
//! and operates the module. By itself the module can not be used as a
//! 'wireless wire', although such a mode could be implemented in the
//! microcontroller. The interface between the RFM73 and the microcontroller
//! is a standard 4-PIN SPI interface (MISO, MOSI, CLCK, CSN) plus a CE
//! (Chip Enable) pin. The module also provides an IRQ pin that could be used
//! to speed up the detection of certain events within the module. 
//! The library does not used this pin. 
//! The datahseet seems to claim that the SPI input pins are 5V-tolerant, 
//! but experiments have shown that this is not the case. 
//!
//! An RFM73 module operates in the 2.4 GHz band. 
//! Within that band a frequency can be selected in 1 MHz steps. 
//! RFM73 modules can communicate only when they use the same frequency.
//!
//! The RFM73 module operates on a packet basis. 
//! Each packet has a destination address.
//! By default the address is a 5 byte value, but a 4 or 3 byte
//! address can also be configured. 
//! For succesfull communication the RFM73 modules that are involved must
//! be configures for the same address length, and the receiving
//! module(s) must have one of their so-called pipes configured
//! for the address used by the transmitter.
//!
//! An RFM73 contains 6 so-called data pipes that can receive packages.
//! Each pipe has an address, and will receive only messages
//! for that address.
//! Pipes 0 and 1 have a full (up to 5 bytes) address. 
//! For pipes 2 .. 6 only the least significant byte can be
//! configured, the other bytes are copied from the address of pipe 1.
//! The length of the address (3, 4 or 5 bytes) is the same for 
//! transmission and for all 6 receive data pipes.
//!
//! A packet contains 1 .. 32 user data bytes. 
//! The packet length can either be fixed or flexible.
//! When the packet length is fixed each receiving pipe is configured
//! with its packet length. The length of a package that is sent is 
//! determined by the number of bytes written into the transmit buffer,
//! and it it must match the length configured for the receiving pipe.
//! When the packet length is flexible the length of a package is again
//! determined by the number of bytes written into the transmit, but in
//! this case this number is transmitted in the message, and on the
//! receiving side it can be retrieved by the R_RX_PL_WID command
//! (rfm73_channel_payload_size function).
//!
//! The simple way to send a package is without acknowledgement.
//! The RFM73 just transmits the package and considers it done.
//! It can be received by zero, one, or multiple RFM73 modules, but
//! the sending modules does not care and has no way of knowing.
//! For this simple mode of communication the involved RFM73's
//! - must be configured for the same channel frequency
//! - must use the same address length
//! - the receiving RFM73 must have a receive pipe configured
//!   for the same address as used by the transmitting RFM73
//!
//! Alternatively a package can be sent with acknowledgement and
//! (if necessary) retransmission. 
//! In this case the RFM73 will send the message, and consider it
//! done only when it receives an acknowledge for it.
//! If it does not receive an acknowledge within a fixed amount of
//! time it will re-transmit the package, up to a set maximum.
//! A receiving RFM73 can automatically send an acknowledgement
//! for a successfully received package. 
//! For this to work the same requirements as for simple unacknowledged
//! communicatiobn apply, and additionally on both RFM73's
//! - CRC must be enabled (length 1 or 2) on both modules
//! - receive data pipe 0 must be configured for the same address 
//!   as the module uses for transmitting
//!
//! The automatic retransmission provided by the RFM73 uses a fixed
//! retransmission timout, which is probly fine for correcting an 
//! occasional packet loss due to interference from other 2.4 GHz sources,
//! but it is potentially inefective when the interference is caused by
//! another RFM73 that uses the same timout! In such cases the 
//! microcontroller should implemnet its own retransmission scheme,
//! with for instance a random timeout, and maybe an exponential 
//! backoff.
//! 
//
//***************************************************************************//

//***************************************************************************//
//
//! \page hardware RFM73 hardware interface
//!
//! \image html smd-pins.jpg
//! \image rtf smd-pins.jpg
//!
//! The RFM73 uses a standard 4-wire SPI interface.
//! It also provides an active low interrupt pin, which could be used to
//! avoid polling. This library does not use the interrupt pin.
//! The RFM73 also has a CE (chip enable) input, which must be de-asserted
//! to put the chip in standby or power-down mode, and must be cycled
//! to switch between receive and transmit mode. Hence the interface
//! uses 5 data pins, plus ground and power (3.3V):
//! - GND : common ground
//! - VDD : 3.3V power
//! - CE : Chip Enable, active high, microcontoller output
//! - CSN : Chip Select Negative, active low, microcontoller output
//! - SCK : Serial ClocK, microcontoller output
//! - MOSI : Master Out Slave In, microcontoller output
//! - MISO : Master In Slave Out, microcontoller input
//! - IRQ : Interrupt ReQuest, not used
//!
//! When the microcontroller operates at 3.3 Volt (or lower, 
//! the RFM73 datasheet claims operation down to 1.9 Volt) all lines, 
//! including power,  
//! can be connected directly to the microcontroller. 
//! If you are experimenting and want to protect yourself against
//! problems when you accidentally configure the pin connected to MISO
//! as output, you could insert a suitable series resistor in this line.
//! 2k2 seems to be a good choice.
//!
//! When the microcontroller operates at 5 Volt there are three possible issues:
//! - power : the RFM73 operates at 1.9 .. 3.3 Volt, so the 5 Volt must somehow
//!   be reduced to a voltage within this range
//! - data from the microcontroller to the RFM73 : although the datasheet seems to
//!   claim that the inputs are 5V tolerant, this does not seem to be the case.
//!   You will have to convert the 5V outputs of your microcontroller down
//!   to 3.3V.
//! - data from RFM73 to the microcontroller : in most cases this will not 
//!   be a problem, but you might want to check the minimum voltage required
//!   by your microcontroller to reliably detect a logic 1. For instance a 
//!   PIC requires ~ 2.0 Volt on a TTL level input, but 0.8 * VDD on a
//!   Schmitt Trigger input! And you must consider this at the maximum VDD 
//!   for the microcontroller, which can be 5.5 Volt when delivered by 
//!   an 7805 or an USB port.
//!
//! There are various ways to create a 3.3 Volt supply for the RFM73 from a
//! 5 Volt supply. I prefer to use a low-drop low-quiecent
//! current 3.3 Volt linear regulator. Read the datasheet of the regulator
//! carefully: some put very stringent requirements on the value and impedance
//! of the decoupling capacitors. My favourite is the Microchip MCP1702, 
//! which has a low quiecent current (5 uA maximum), and is stable with 
//! only 1 uF input an output capacitors of any type 
//! (ceramic, tantalum, aluminium electrolytic).
//!
//! TBW: picture
//!
//! A crude way to create the power for the RFM73 is to use a 
//! resistor divider. I would do this only in an experimental
//! setup on my desk, never in a final product.
//! The source impedance of the divider causes a drop in the 
//! voltage when the RFM73 uses more current.
//! This drop can be reduced by lowering the resistor values, but at
//! the cost of a higher current through the resistors. The RFM73
//! can operate down to 1.9 Volt, but at that level the microcontroller 
//! might not reliably recognise a logic 1 from the RFM73. Another issue
//! is the dissipation in the resistors. The circuit below is a compromise.
//! It uses three equal-valued resistors because I don't stock many
//! different resistor values.
//! The idle current through the resistors is 83 mA at 5.5 Volt, in
//! this situation the RFM73 gets 3.7 Volt. That is slightly high, but
//! probably not a big problem.
//! When the RFM73 draws its maximum current of 23 mA when the 
//! microcontroller's power is at 4.5 Volt the RFM73 still gets 2.6 Volt. 
//! You might want to double-check that the microcontroller accepts ~ 2 Volt
//! as a logic 1. 
//!
//! TBW: picture
//! 
//
//***************************************************************************//




#include "radio_lib.h"


/* 
* 	defines and tables needed by library	
*/

// RFM73 SPI read and write commands
#define RFM73_CMD_READ_REG            0x00
#define RFM73_CMD_WRITE_REG           0x20

//interrupt status
#define STATUS_RX_DR    0x40
#define STATUS_TX_DS    0x20
#define STATUS_MAX_RT   0x10

#define STATUS_TX_FULL  0x01

//FIFO_STATUS
#define FIFO_STATUS_TX_REUSE  0x40
#define FIFO_STATUS_TX_FULL   0x20
#define FIFO_STATUS_TX_EMPTY  0x10

#define FIFO_STATUS_RX_FULL   0x02
#define FIFO_STATUS_RX_EMPTY  0x01



// magic bank1 register initialization values
const unsigned long Bank1_Reg0_13[] = {
   0xE2014B40,
   0x00004BC0,
   0x028CFCD0,
   0x41390099,
   0x1B8296D9, 
   0xA67F0624,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00127300,
   0x36B48000 };   
   
// more magic bank1 register initialization values
const unsigned char Bank1_Reg14[] = {
   0x41, 0x20, 0x08, 0x04, 0x81, 0x20, 0xCF, 0xF7, 0xFE, 0xFF, 0xFF };   
   


// Bank0 register initialization values
#define BANK0_ENTRIES 10
const unsigned char Bank0_Reg[ BANK0_ENTRIES ][ 2 ]={
   {  0, 0x0F }, // receive, enabled, CRC 2, enable interupts
   {  1, 0x3F }, // auto-ack on all pipes enabled
   {  2, 0x07 }, // Enable pipes 0 and 1 and 2
   {  3, 0x03 }, // 5 bytes addresses
   {  4, 0x27 }, // 0x27- 750us, 7 times | 0xff - auto retransmission delay 4 ms, 15 times
   {  5, 0x0A }, // channel 10
   {  6 ,0x2F }, // 2MBit, power TX 5dBm, LNA gain high - 0x27 | 0x07 - data rate 1Mbit, power 5dbm, LNA gain high
   {  7, 0x07 }, // why write this at all?? but seems required to work...
   {  8, 0x00 }, // clear Tx packet counters
   { 23, 0x00 }, // fifo status
};


   // default receive address data pipe 0:
   // just a bunch of bytes, nothing magical
const unsigned char RX0_Address[]={ 0x34, 0x43, 0x10, 0x10, 0x01 };
const unsigned char RX2_Address[]={ 0x12, 0x43, 0x10, 0x10, 0x01 };


// Set or clear CE pin
void RFM73_CE( struct Radio_TypeDef *_handler, uint8_t _val ) {
	if ( _val == 0 ) 
		_handler->A_SPI_gpio_port->BSRRH |= _handler->A_SPI_CE_pin ;
	else 
		_handler->A_SPI_gpio_port->BSRRL |= _handler->A_SPI_CE_pin ;
}


// Set or clear CSN pin
void RFM73_CSN( struct Radio_TypeDef *_handler, uint8_t _val ) {
	if ( _val == 0 ) 
		_handler->A_SPI_gpio_port->BSRRH |= _handler->A_SPI_CSN_pin ;
	else 
		_handler->A_SPI_gpio_port->BSRRL |= _handler->A_SPI_CSN_pin ;
	
}


// This funtion send data / read by SPI - full duplex

// rfm73_SPI_RW will be renamed to rfm73_SPI_send ( in future, while everything will work )
uint8_t rfm73_SPI_RW ( R_SPI_HandleTypeDef *_spiH, uint8_t _data ) {
	
	//wait for empty TX buffer
	while( !(_spiH->Instance->SR & SPI_SR_TXE) ) ;
	//push data to TX buffer
	_spiH->Instance->DR = (uint16_t)_data;

	// wait for full RX buffer
	while( !(_spiH->Instance->SR & SPI_SR_RXNE) ) ;
	
	// read RX buffer
	return (uint8_t)_spiH->Instance->DR;
	
}


// Write only to SPI - don't care about rx buffer
// Not good idea with that - problem with data mixing occurred
/*
void rfm73_SPI_WO ( R_SPI_HandleTypeDef *_spiH, uint8_t _data ) {
	
	uint8_t temp ;
	
	//wait for empty TX buffer
	while( !(_spiH->Instance->SR & SPI_SR_TXE) ) ;
	//push data to TX buffer
	_spiH->Instance->DR = (uint16_t)_data;
	
	// flush rx buffer
	if( _spiH->Instance->SR & SPI_SR_RXNE )
		temp = (uint8_t)_spiH->Instance->DR ;
	
}
*/




void rfm73_register_write( struct Radio_TypeDef * _radioH, 
													unsigned char reg,
													unsigned char value )
{
   if( reg < RFM73_CMD_WRITE_REG ){
      reg |= RFM73_CMD_WRITE_REG;      
   }  
   RFM73_CSN( _radioH, 0 );      // CSN low, init SPI transaction
   (void)rfm73_SPI_RW( _radioH->spi_inst, reg );    		 // select register
	 (void)rfm73_SPI_RW( _radioH->spi_inst, value );  		 // ..and write value to it..
   RFM73_CSN( _radioH, 1 );      // CSN high again
}





unsigned char rfm73_register_read( struct Radio_TypeDef * _radioH,
																	 unsigned char reg )
{
   unsigned char value;
   if( reg < RFM73_CMD_WRITE_REG ){
      reg |= RFM73_CMD_READ_REG;       
   }  
   RFM73_CSN( _radioH, 0 );     // CSN low, initialize SPI communication...
   (void)rfm73_SPI_RW( _radioH->spi_inst, reg );   // Select register to read from..
   value = rfm73_SPI_RW( _radioH->spi_inst, 0 );   // ..then read register value
   RFM73_CSN( _radioH, 1 );               // CSN high, terminate SPI communication
   return value;                // return register value
}



void rfm73_bank( struct Radio_TypeDef * _radioH,
								 unsigned char b )
{
   unsigned char st = 0x80 & rfm73_register_read( _radioH, RFM73_REG_STATUS );
   if(( st &&( b == 0 )) || (( st == 0 ) && b )){
      rfm73_register_write( _radioH, RFM73_CMD_ACTIVATE, 0x53 );
   }
}




void rfm73_buffer_write(
	 struct Radio_TypeDef * _radioH,
   char reg, 
   const unsigned char pBuf[], 
   unsigned char length 
){
   unsigned char i;
   if( reg < RFM73_CMD_WRITE_REG ){
      reg |= RFM73_CMD_WRITE_REG;      
   }  
   RFM73_CSN( _radioH, 0 );            // Set CSN low, init SPI tranaction
   rfm73_SPI_RW( _radioH->spi_inst, reg );          // Select register to write tio write
   
	 for( i = 0; i < length; i++ ){       // write all bytes in buffer(*pBuf)
      rfm73_SPI_RW( _radioH->spi_inst, pBuf[ i ]);  // write one byte
   }   
	 
   RFM73_CSN( _radioH, 1 );            // Set CSN high again
}





void rfm73_buffer_read(
	 struct Radio_TypeDef * _radioH,
   unsigned char reg, 
   unsigned char pBuf[], 
   unsigned char length 
){
   unsigned char i;
   if( reg < RFM73_CMD_WRITE_REG ){
      reg |= RFM73_CMD_READ_REG;       
   }  
   RFM73_CSN( _radioH, 0 );            // Set CSN 0
	 // Select register to write
   (void)rfm73_SPI_RW( _radioH->spi_inst, reg );         
   for( i = 0; i < length; i++ ){      // read all bytes
		 // read one byte from RFM73
     pBuf[ i ] = rfm73_SPI_RW( _radioH->spi_inst, 0 );
   }
   RFM73_CSN( _radioH, 1 );                     // Set CSN high again
}






void rfm73_receive_address_p0( struct Radio_TypeDef * _radioH, const unsigned char address[ 5 ] ){
   rfm73_buffer_write( _radioH, RFM73_REG_RX_ADDR_P0, address, 5 );  
}

void rfm73_receive_address_p1( struct Radio_TypeDef * _radioH, const unsigned char address[ 5 ] ){
   rfm73_buffer_write( _radioH, RFM73_REG_RX_ADDR_P1, address, 5 );  
}


void rfm73_transmit_address( struct Radio_TypeDef * _radioH, const unsigned char address[] ){
   rfm73_buffer_write( _radioH, RFM73_REG_TX_ADDR, address, 5 );   
} 


void rfm73_receive_address_pn(
	struct Radio_TypeDef * _radioH,
	unsigned char channel,
	unsigned char address 
){
   rfm73_register_write( _radioH, RFM73_REG_RX_ADDR_P0 + channel, address ); 
}

unsigned char rfm73_retransmit_count( struct Radio_TypeDef * _radioH ){
   return rfm73_register_read( _radioH, RFM73_REG_OBSERVE_TX ) & 0x0F;   
}

unsigned char rfm73_lost_packets_count( struct Radio_TypeDef * _radioH ){
   return ( rfm73_register_read( _radioH, RFM73_REG_OBSERVE_TX ) >> 4 ) & 0x0F;   
}




void rfm73_channel_payload_size( 
		struct Radio_TypeDef * _radioH, 
		unsigned char channel,
		unsigned char size 
) {
   unsigned char val;
   if( size > 32 ){
      size = 32;
   }
   val = rfm73_register_read( _radioH, RFM73_REG_DYNPD );  
   if( size == 0 ){
      val |= 1 << channel;
   } else {
      val &= ~ ( 1 << channel );
   }    
   rfm73_register_write( _radioH, RFM73_REG_DYNPD, val );
   rfm73_register_write( _radioH, RFM73_REG_RX_PW_P0 + channel, size );  
}






// initialize bank1 like in the example code.
// don't ask why, just do it
void rfm73_init_bank1( struct Radio_TypeDef * _radioH ){
   unsigned char i, j;
   unsigned char WriteArr[ 12 ];
        
   rfm73_bank( _radioH, 1 );
   
   for( i = 0; i <= 8; i++ ){ //reverse!
      for( j = 0; j < 4; j++ ){
         WriteArr[ j ]=( Bank1_Reg0_13[ i ]>>(8*(j) ) )&0xff;
      }  
      rfm73_buffer_write( _radioH, i, WriteArr, 4 );
   }

   for( i = 9; i <= 13; i++ ){
      for( j = 0; j < 4; j++ ){
         WriteArr[ j ]=( Bank1_Reg0_13[ i ]>>(8*(3-j) ) )&0xff;
      }
      rfm73_buffer_write( _radioH, i, WriteArr, 4 );
   }

   rfm73_buffer_write( _radioH, 14, Bank1_Reg14, 11 );

   //toggle REG4<25,26>
   for(j=0;j<4;j++){
      WriteArr[ j ]=( Bank1_Reg0_13[ 4 ]>>(8*(j) ) )&0xff;
   } 

   WriteArr[ 0 ] = WriteArr[ 0 ] | 0x06;
   rfm73_buffer_write( _radioH, 4, WriteArr, 4);

   WriteArr[ 0 ] = WriteArr[ 0 ] & 0xf9;
   rfm73_buffer_write( _radioH, 4, WriteArr,4);

   rfm73_bank( _radioH, 0 );
}






void rfm73_mode_receive( struct Radio_TypeDef * _radioH ){
   unsigned char value;
   
   // flush receive queue 
   rfm73_register_write( _radioH, RFM73_CMD_FLUSH_RX, 0 );
   
   // clear interrupt status
   value = rfm73_register_read( _radioH, RFM73_REG_STATUS );
   rfm73_register_write( _radioH, RFM73_REG_STATUS ,value );
   
   // switch to receive mode
   RFM73_CE( _radioH, 0 );
   value = rfm73_register_read( _radioH, RFM73_REG_CONFIG );
   value |= 0x01; // set RX bit
   value |= 0x02; // set PWR_UP bit
   rfm73_register_write( _radioH, RFM73_REG_CONFIG, value );
   RFM73_CE( _radioH, 1 );
}





unsigned char rfm73_is_present( struct Radio_TypeDef * _radioH ){
   unsigned char st1, st2; 
   st1 = rfm73_register_read( _radioH, RFM73_REG_STATUS );
   rfm73_register_write( _radioH, RFM73_CMD_ACTIVATE, 0x53 );
   st2 = rfm73_register_read( _radioH, RFM73_REG_STATUS );
   rfm73_register_write( _radioH, RFM73_CMD_ACTIVATE, 0x53 );
   return (( (st1 & 0x80) ^ (st2 & 0x80) ) == 0x80 );
}



void rfm73_mode_standby( struct Radio_TypeDef * _radioH ){
   unsigned char value;
   RFM73_CE( _radioH, 0 );
   value = rfm73_register_read( _radioH, RFM73_REG_CONFIG );
   value |= 0x02; // set PWR_UP bit
   rfm73_register_write( _radioH, RFM73_REG_CONFIG, value );
}

void rfm73_mode_powerdown( struct Radio_TypeDef * _radioH ){
   unsigned char value;
   RFM73_CE( _radioH, 0 );
   value = rfm73_register_read( _radioH, RFM73_REG_CONFIG );
   value &= 0xFD; // clear PWR_UP bit
   rfm73_register_write( _radioH, RFM73_REG_CONFIG, value );
}



void rfm73_mode_transmit( struct Radio_TypeDef * _radioH ){
   unsigned char value;
   
   // flush transmit queue
   rfm73_register_write( _radioH, RFM73_CMD_FLUSH_TX, 0);
   
   // clear interrupt status
   value = rfm73_register_read( _radioH, RFM73_REG_STATUS );
   rfm73_register_write(  _radioH, RFM73_REG_STATUS ,value );
   
   // switch to transmit mode
   RFM73_CE( _radioH, 0 );
   value = rfm73_register_read( _radioH, RFM73_REG_CONFIG );
   value &= 0xFE; // clear RX bit
   value |= 0x02; // set PWR_UP bit
   rfm73_register_write( _radioH, RFM73_REG_CONFIG, value );
   RFM73_CE( _radioH, 1 );
}





void rfm73_channel( struct Radio_TypeDef * _radioH, unsigned char ch ){
  // MSB must be 0
  rfm73_register_write( _radioH, RFM73_REG_RF_CH, ch & 0x7E );
}





void rfm73_air_data_rate( struct Radio_TypeDef * _radioH, unsigned char rate ){
   unsigned char value; 
   RFM73_CE( _radioH, 0 );
   value = rfm73_register_read( _radioH, RFM73_REG_RF_SETUP );
   value &= 0x07;
   if( rate == 0 ){
      value |= 0x20;  
   }
   if( rate > 1 ){
      value |= 0x08;  
   }
   rfm73_register_write( _radioH, RFM73_REG_RF_SETUP, value );
   RFM73_CE( _radioH, 1 );
}

void rfm73_crc_length( struct Radio_TypeDef * _radioH, unsigned char len ){
   unsigned char val;
   if( len > 2 ){
      len = 2;
   }   
   val = rfm73_register_read( _radioH, RFM73_REG_CONFIG );
   if( len == 0 ){
      val &= 0xF3;
      rfm73_register_write( _radioH, RFM73_REG_EN_AA, 0 );
      rfm73_register_write( _radioH, RFM73_REG_CONFIG, val );
   } else {
      rfm73_register_write( _radioH, RFM73_REG_EN_AA, 0x3F );
      val &= 0xFB;
      if( val == 2 ){
         val |= 0x04; 
      }   
      rfm73_register_write( _radioH, RFM73_REG_CONFIG, val );
   }  
}

void rfm73_address_length( struct Radio_TypeDef * _radioH, unsigned char len ){
   if( len > 5 ){
      len = 5;
   }
   if( len < 3 ){
      len = 3;    
   }
   rfm73_register_write( _radioH, RFM73_REG_SETUP_AW, len - 2  );   
}

unsigned char rfm73_transmit_fifo_full( struct Radio_TypeDef * _radioH ){
  unsigned char s;
  s = rfm73_register_read( _radioH, RFM73_REG_FIFO_STATUS );
  return ( s & FIFO_STATUS_TX_FULL ) != 0;
}
    



unsigned char rfm73_receive_fifo_empty( struct Radio_TypeDef * _radioH ){
  unsigned char s;
  s = rfm73_register_read( _radioH, RFM73_REG_FIFO_STATUS );
  return ( s & FIFO_STATUS_RX_EMPTY ) != 0;
}




void rfm73_pipe_autoack(
		struct Radio_TypeDef * _radioH,
		unsigned char pipe,
		unsigned char enabled
){
   unsigned char val = rfm73_register_read( _radioH, RFM73_REG_EN_AA ); 
   if( pipe > 5 ){ 
      pipe = 5; 
   }   
   if( enabled ){
      val |= 1 << pipe;
   } else {
      val &= ~ ( 1 << pipe );
   }
   rfm73_register_write( _radioH, RFM73_REG_EN_AA, val ); 
}



void rfm73_pipe_enable(
		struct Radio_TypeDef * _radioH,
		unsigned char pipe,
		unsigned char enabled 
){
   unsigned char val = rfm73_register_read( _radioH, RFM73_REG_EN_RXADDR ); 
   if( pipe > 5 ){ 
      pipe = 5; 
   }   
   if( enabled ){
      val |= 1 << pipe;
   } else {
      val &= ~ ( 1 << pipe );
   }
   rfm73_register_write( _radioH, RFM73_REG_EN_RXADDR, val ); 
}






void rfm73_lost_packets_reset( struct Radio_TypeDef * _radioH ){
   unsigned char val = rfm73_register_read( _radioH, RFM73_REG_RF_CH );  
   rfm73_register_write( _radioH, RFM73_REG_RF_CH, val );    
}


void rfm73_retransmit_delay_attempts(
			struct Radio_TypeDef * _radioH,
			unsigned char d,
			unsigned char n 
){
   rfm73_register_write( _radioH,
      RFM73_REG_SETUP_RETR, 
			( n & 0x0F ) | (( d & 0x0F ) << 4 ));
}


void rfm73_lna_high( struct Radio_TypeDef * _radioH ){
   unsigned char val = rfm73_register_read( _radioH, RFM73_REG_RF_SETUP ); 
   val |= 0x01;
   rfm73_register_write( _radioH, RFM73_REG_RF_SETUP, val ); 
}

void rfm73_lna_low( struct Radio_TypeDef * _radioH ){
   unsigned char val = rfm73_register_read( _radioH, RFM73_REG_RF_SETUP ); 
   val &= 0xFE;
   rfm73_register_write( _radioH, RFM73_REG_RF_SETUP, val ); 
}


void rfm73_power( struct Radio_TypeDef * _radioH, unsigned char level ){
   if( level > 3 ){ 
       level = 3; 
   }
   RFM73_CE( _radioH, 0 );
   unsigned char val = rfm73_register_read( _radioH, RFM73_REG_RF_SETUP ); 
   val &= 0x09; 
   val |= 0x30;
   val |= ( level << 1 );
   rfm73_register_write( _radioH, RFM73_REG_RF_SETUP, val ); 
   RFM73_CE( _radioH, 1 );
}





void rfm73_transmit_message(
	 struct Radio_TypeDef * _radioH,
   const unsigned char buf[],
   unsigned char length
){
   if( length > 32 ){
      length = 32;
   }
   rfm73_buffer_write( _radioH, RFM73_CMD_W_TX_PAYLOAD, buf, length );
}
    



void rfm73_transmit_message_once(
	 struct Radio_TypeDef * _radioH,
   const unsigned char buf[],
   unsigned char length
){
   if( length > 32 ){
      length = 32;
   }
   rfm73_buffer_write( _radioH, RFM73_CMD_W_TX_PAYLOAD_NOACK, buf, length );
}




unsigned char rfm73_receive_next_pipe( struct Radio_TypeDef * _radioH ){
   unsigned char status = rfm73_register_read( _radioH, RFM73_REG_STATUS ); 
   return ( status >> 1 ) & 0x07;
}




unsigned char rfm73_receive_next_length( struct Radio_TypeDef * _radioH ){
   return rfm73_register_read( _radioH, RFM73_CMD_R_RX_PL_WID );
}





unsigned char rfm73_receive(
	 struct Radio_TypeDef * _radioH,
   unsigned char * pipe,
   unsigned char buf[],
   unsigned char * length
){
   unsigned char p = rfm73_receive_next_pipe( _radioH );
   if( p == 0x07 ){
      return 0;
   }   
   * pipe = p;
   * length = rfm73_receive_next_length( _radioH );
   rfm73_buffer_read( _radioH, RFM73_CMD_R_RX_PAYLOAD, buf, * length );
   return 1;
}




void rfm73_wait_ms( uint32_t _delay ) {
	uint32_t starttimer = HAL_GetTick();
	while( (uint32_t)( HAL_GetTick() - starttimer ) <= _delay);
}


void rfm73_init( struct Radio_TypeDef * _radioH ) {
	unsigned char i;
   
	 // Disable SPI RX not empty interrpt
		_radioH->spi_inst->Instance->CR2 &= ~(SPI_CR2_RXNEIE);
	
   //RFM73_PIN_DIRECTION;

   RFM73_CE( _radioH, 0 );
   RFM73_CSN( _radioH, 1 );
		
	 // delay at least 50ms.
   // the example code says so, but why??
   rfm73_wait_ms( 100 );
	
	/*
   RFM73_SCK( 0 );
   RFM73_MOSI( 0 );
	*/
	
   // write array of default init settings
   rfm73_bank( _radioH, 0 );
   for( i = 0; i < BANK0_ENTRIES; i++ ){
     rfm73_register_write( _radioH, Bank0_Reg[ i ][0], Bank0_Reg[ i ][1] );
   }

   rfm73_receive_address_p0( _radioH, RX0_Address );
   rfm73_receive_address_p1( _radioH, RX0_Address );
	 rfm73_receive_address_pn( _radioH, 2, RX2_Address[0] );
   rfm73_transmit_address( _radioH, RX0_Address );

   // enable the extra features
   i = rfm73_register_read( _radioH, 29);
   if( i == 0 ){ // only when the extra features are not yet activated!
      rfm73_register_write( _radioH, RFM73_CMD_ACTIVATE, 0x73 );
   }
   
   // select dynamic payload length data pipe5\4\3\2\1\0
   rfm73_register_write( _radioH, 28, 0x3F ); 
   
   // select Dynamic Payload Length, Payload with ACK, W_TX_PAYLOAD_NOACK
   rfm73_register_write( _radioH, 29, 0x07 );  
   
   // dynamic payload sizing on channels 0 and 1
   rfm73_channel_payload_size( _radioH, 0, 0 );
   rfm73_channel_payload_size( _radioH, 1, 0 );

   rfm73_init_bank1( _radioH );
	 
	 rfm73_wait_ms( 100 ); 
	 
   rfm73_mode_receive( _radioH );
	 
	 // Enable SPI RX not empty interrpt
	 //	_radioH->spi_inst->Instance->CR2 |= (SPI_CR2_RXNEIE);
}



void _rfm73_analyze( struct Radio_TypeDef * _radioH ){
	
	uint8_t clr_int = 0;
	
	
	// clear IRQ flag
	_radioH->status &= ~(RFM73_IRQ_OCR_MASK) ;
	
	// Read status register
	_radioH->buff_stat = rfm73_register_read( _radioH, RFM73_REG_STATUS );

	
	
	// If data was send properly ( and / or was delivered ( Acknowledge mode ) packet to destination )
	if ( _radioH->buff_stat & (1 << 5 ) ) {
			// TO DO STH.
			clr_int |= (1 << 5) ;
	}
	
	// If data was not send properly - maximum retransmiton times reached
	if ( _radioH->buff_stat & (1 << 4 ) ) {
			
			__DBG_SEND_CHAR('-');
			__DBG_SEND_CHAR('M');
			__DBG_SEND_CHAR('-');
			clr_int |= (1 << 4) ;
	}
	

	// Any data are waiting in RX FIFO? 
	if (_radioH->buff_stat & (1 << 6 )) {
		// if yes - then read them

				
		if ( !(_radioH->status & RFM73_D_READING_MASK) ){
				// If there is no active data transfer
				
				// Turn off SPI RX interrupt - for precaution
				_radioH->spi_inst->Instance->CR2 &= ~(SPI_CR2_RXNEIE) ;
			
				// clear flags in RFM73 before next step
				clr_int |= ( 1 << 6 ) ;
				rfm73_register_write( _radioH, RFM73_REG_STATUS, clr_int ); // clear ints
				
			
				// get pipe number
				_radioH->pipe = ( _radioH->buff_stat >> 1 ) & 0x07 ;
			
				// get length ( B ) of data
				_radioH->buffer_maxl = rfm73_register_read( _radioH, RFM73_CMD_R_RX_PL_WID );
				// set buffer[] index to first (0)
				_radioH->buffer_cpos = 0 ;
			
			
			
				RFM73_CSN( _radioH, 0 );            				// Set CSN 0 - start data transmition ( the end is in SPI3 IRQ func. )
				// Select register to write
				(void)rfm73_SPI_RW( _radioH->spi_inst, RFM73_CMD_R_RX_PAYLOAD );         
				
				// Enable SPI interrupt
				NVIC_ClearPendingIRQ( _radioH->spi_irqn );
				_radioH->spi_inst->Instance->CR2 |= SPI_CR2_RXNEIE ;
					
				// Set flag that allow SPI interrupt to read data: 
				_radioH->status |= RFM73_D_READING_MASK ;
				
				// send info, that want to read one byte from RFM73
				while( !(_radioH->spi_inst->Instance->SR & SPI_SR_TXE) ) ;			
				_radioH->spi_inst->Instance->DR = 0;				// received data will triger proper SPI interrupt
				
		} 
		else {
			// there is active data transfer - need to inform main that new data are waiting in RXFIFO
			_radioH->status |= RFM73_D_PENDING_MASK ;
		}
	} 
	else {
		
		// Clear flags only when there is no active data transfer !!!!
		if ( !(_radioH->status & RFM73_D_READING_MASK) ) 
			rfm73_register_write( _radioH, RFM73_REG_STATUS, clr_int ) ;
		
	}
	
	
	
}


void rfm73_check( struct Radio_TypeDef * _radioH ) {
	
		#ifdef __DBG_ITM
			uint8_t temp8;
		#endif
		
		// if IRQ interrupt was received
		/// *******// Probably good idea is to test IRQpin if it has low level ( becouse probably rfm73 can keep low level
		// when have even one interrupt flag set ( in his status register )
		if ( (_radioH->status & RFM73_IRQ_OCR_MASK) ) {
			
			__DBG_SEND_CHAR('A');
			
			// Perform test his status register
			_rfm73_analyze( _radioH );
			
		}
		
		
		
		// If data from RFM73 module was read properly
		if ( _radioH->status & RFM73_D_READY_MASK ) {

			// Clear library flags - ready for another data
			_radioH->status &= ~( RFM73_D_READY_MASK | RFM73_D_READING_MASK );
			
			// Clear int. flags in module: 
			//rfm73_register_write( &radio2, RFM73_REG_STATUS, 0x70 );  //clear ints
			
			
			#ifdef __DBG_ITM
			for ( temp8 = 0 ; temp8 < _radioH->buffer_maxl ; temp8++ ) 
				ITM_SendChar( _radioH->buffer[temp8] ) ;
			#endif

		}
		
		
	
}





void rfm73_rx_interrupt_handle ( struct Radio_TypeDef * _radioH ) {
	
	if ( (_radioH->status & RFM73_D_READING_MASK) && 
			 !(_radioH->status & RFM73_D_READY_MASK)
		 ) {
				
				//__DBG_SEND_CHAR('I');
				
				// Copy just read data to buffer
				_radioH->buffer[ _radioH->buffer_cpos ] = _radioH->spi_inst->Instance->DR ;
				
				_radioH->buffer_cpos++ ;
				
				// All bytes was read? 
				if( _radioH->buffer_cpos >= _radioH->buffer_maxl ) {
					// Inform main that whole data was read
					_radioH->status |= RFM73_D_READY_MASK ;
					
					RFM73_CSN( _radioH, 1 ); 				// End of transmition
					
					// Mask source of this interrpt ( disable )
					_radioH->spi_inst->Instance->CR2 &= ~(SPI_CR2_RXNEIE) ;
				}
				else {
					// still some bytes are waitng in RFM73 RX FIFO
					
					// send infomation to RFM73 to send next byte
					// wait for empty TX buffer
					while( !(_radioH->spi_inst->Instance->SR & SPI_SR_TXE) ) ;			
					_radioH->spi_inst->Instance->DR = 0;				// received data will triger that interrupt
					
				}
			} 
			// else option is hard fault?  Could it happen?

}

