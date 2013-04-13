/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis
  Modified 2012 by Fredrik Hubinette

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id: wiring.h 249 2007-02-03 16:52:51Z mellis $
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define TX_RX_LED_INIT		DDRE |= (1<<7)
#define TXLED0			0
#define TXLED1			0
#define RXLED0			PORTE |= (1<<7)
#define RXLED1			PORTE &= ~(1<<7)

static const uint8_t SDA = 0;
static const uint8_t SCL = 1;

// Map SPI port to 'new' pins D14..D17
static const uint8_t SS   = 20;
static const uint8_t MOSI = 22;
static const uint8_t MISO = 23;
static const uint8_t SCK  = 21;

// Mapping of analog pins as digital I/O
// A6-A11 share with digital pins
static const uint8_t A0 = 38;   // F0
static const uint8_t A1 = 39;   // F1
static const uint8_t A2 = 40;   // F2
static const uint8_t A3 = 41;   // F3
static const uint8_t A4 = 42;   // F4
static const uint8_t A5 = 43;   // F5
static const uint8_t A6 = 44;	// F6
static const uint8_t A7 = 45;	// F7

#define analogInputToDigitalPin(p)  ((p < 16) ? (p) + 54 : -1)

// Pins below still needs to be configured - Hubbe.

#define digitalPinToPCICR(p)    ((((p) >= 8 && (p) <= 11) || ((p) >= 14 && (p) <= 17) || ((p) >= A8 && (p) <= A10)) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) 0
#define digitalPinToPCMSK(p)    ((((p) >= 8 && (p) <= 11) || ((p) >= 14 && (p) <= 17) || ((p) >= A8 && (p) <= A10)) ? (&PCMSK0) : ((uint8_t *)0))
#define digitalPinToPCMSKbit(p) ( ((p) >= 8 && (p) <= 11) ? (p) - 4 : ((p) == 14 ? 3 : ((p) == 15 ? 1 : ((p) == 16 ? 2 : ((p) == 17 ? 0 : (p - A8 + 4))))))

//	__AVR_ATmega32U4__ has an unusual mapping of pins to channels
extern const uint8_t PROGMEM analog_pin_to_channel_PGM[];
#define analogPinToChannel(P)  ( pgm_read_byte( analog_pin_to_channel_PGM + (P) ) )

#ifdef ARDUINO_MAIN


// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
// Note PA == 1, PB == 2, etc. (GAH!)
const uint16_t PROGMEM port_to_mode_PGM[] = {
  NOT_A_PORT,
  (uint16_t) &DDRA,
  (uint16_t) &DDRB,
  (uint16_t) &DDRC,
  (uint16_t) &DDRD,
  (uint16_t) &DDRE,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
  NOT_A_PORT,
  (uint16_t) &PORTA,
  (uint16_t) &PORTB,
  (uint16_t) &PORTC,
  (uint16_t) &PORTD,
  (uint16_t) &PORTE,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
  NOT_A_PORT,
  (uint16_t) &PINA,
  (uint16_t) &PINB,
  (uint16_t) &PINC,
  (uint16_t) &PIND,
  (uint16_t) &PINE,
};


#define PIN_DEFS				\
  DEFPIN(PA, 0),				\
  DEFPIN(PA, 1),				\
  DEFPIN(PA, 2),				\
  DEFPIN(PA, 3),				\
  DEFPIN(PA, 4),				\
  DEFPIN(PA, 5),				\
  DEFPIN(PA, 6),				\
  DEFPIN(PA, 7),				\
  DEFPIN(PB, 0),				\
  DEFPIN(PB, 1),				\
  DEFPIN(PB, 2),				\
  DEFPIN(PB, 3),				\
  DEFPIN(PB, 4),				\
  DEFPIN(PB, 5),				\
  DEFPIN(PB, 6),				\
  DEFPIN(PB, 7),				\
  DEFPIN(PC, 0),				\
  DEFPIN(PC, 1),				\
  DEFPIN(PC, 2),				\
  DEFPIN(PC, 3),				\
  DEFPIN(PC, 4),				\
  DEFPIN(PC, 5),				\
  DEFPIN(PC, 6),				\
  DEFPIN(PC, 7),				\
  DEFPIN(PD, 0),				\
  DEFPIN(PD, 1),				\
  DEFPIN(PD, 2),				\
  DEFPIN(PD, 3),				\
  DEFPIN(PD, 4),				\
  DEFPIN(PD, 5),				\
  DEFPIN(PD, 6),				\
  DEFPIN(PD, 7),				\
  DEFPIN(PE, 0),				\
  DEFPIN(PE, 1),				\
  DEFPIN(PE, 2),				\
  DEFPIN(PE, 3),				\
  DEFPIN(PE, 4),				\
  DEFPIN(PE, 5),				\
  DEFPIN(PE, 6),				\
  DEFPIN(PE, 7),


#undef DEFPIN
#define DEFPIN(PORT, PIN) PORT

// TODO(hubbe) figure out all the pins
const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
  PIN_DEFS
};

#undef DEFPIN
#define DEFPIN(PORT, PIN) _BV(PIN)

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
  PIN_DEFS
};

// TOOD(hubbe) figure out timer mapping
#undef DEFPIN
#define DEFPIN(PORT, PIN) NOT_ON_TIMER
const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
  PIN_DEFS
};

const uint8_t PROGMEM analog_pin_to_channel_PGM[12] = {
	7,	// A0				PF7					ADC7
	6,	// A1				PF6					ADC6	
	5,	// A2				PF5					ADC5	
	4,	// A3				PF4					ADC4
	1,	// A4				PF1					ADC1	
	0,	// A5				PF0					ADC0	
	8,	// A6		D4		PD4					ADC8
	10,	// A7		D6		PD7					ADC10
	11,	// A8		D8		PB4					ADC11
	12,	// A9		D9		PB5					ADC12
	13,	// A10		D10		PB6					ADC13
	9	// A11		D12		PD6					ADC9
};

#endif /* ARDUINO_MAIN */
#endif /* Pins_Arduino_h */
