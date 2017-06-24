#ifndef pins_macros_for_arduino_compatibility_h
#define pins_macros_for_arduino_compatibility_h

#include <avr/pgmspace.h>
#include "core_pins.h"

#if defined(__AVR_ATmega32U4__) || defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
const static uint8_t A0 = 38;
const static uint8_t A1 = 39;
const static uint8_t A2 = 40;
const static uint8_t A3 = 41;
const static uint8_t A4 = 42;
const static uint8_t A5 = 43;
const static uint8_t A6 = 44;
const static uint8_t A7 = 45;
#endif

const static uint8_t SS   = 20;
const static uint8_t MOSI = 22;
const static uint8_t MISO = 23;
const static uint8_t SCK  = 21;
const static uint8_t LED_BUILTIN = CORE_LED0_PIN;
const static uint8_t SDA  = 0;
const static uint8_t SCL  = 1;

#define NUM_DIGITAL_PINS                48
#define NUM_ANALOG_INPUTS               8

#define TX_RX_LED_INIT          DDRE |= (1<<7)
#define TXLED0                  0
#define TXLED1                  0
#define RXLED0                  PORTE |= (1<<7)
#define RXLED1                  PORTE &= ~(1<<7)

// This allows CapSense to work.  Do any libraries
// depend on these to be zero?
#define NOT_A_PORT 127
#define NOT_A_PIN 127
#define NOT_AN_INTERRUPT -1

#define digitalPinToPort(P) (P)
#define portInputRegister(P) ((volatile uint8_t *)((int)pgm_read_byte(digital_pin_table_PGM+(P)*2+1)))
#define portModeRegister(P) (portInputRegister(P) + 1)
#define portOutputRegister(P) (portInputRegister(P) + 2)
#define digitalPinToBitMask(P) (pgm_read_byte(digital_pin_table_PGM+(P)*2))
extern const uint8_t PROGMEM digital_pin_table_PGM[];

#define analogInputToDigitalPin(p)  ((p < 16) ? (p) + 54 : -1)
#define digitalPinHasPWM(p)         ((p) == 12 || (p) == 13 || (p) == 14 || (p) == 20 || (p) == 21 || (p) == 22)
#define digitalPinToPCICR(p)    ((((p) >= 8 && (p) <= 11) || ((p) >= 14 && (p) <= 17) || ((p) >= A8 && (p) <= A10)) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) 0
#define digitalPinToPCMSK(p)    ((((p) >= 8 && (p) <= 11) || ((p) >= 14 && (p) <= 17) || ((p) >= A8 && (p) <= A10)) ? (&PCMSK0) : ((uint8_t *)0))
#define digitalPinToPCMSKbit(p) ( ((p) >= 8 && (p) <= 11) ? (p) - 4 : ((p) == 14 ? 3 : ((p) == 15 ? 1 : ((p) == 16 ? 2 : ((p) == 17 ? 0 : (p - A8 + 4))))))


#if defined(__AVR_AT90USB162__)
#define analogInputToDigitalPin(ch)	(-1)
#define digitalPinHasPWM(p)		((p) == 0 || (p) == 15 || (p) == 17 || (p) == 18)
#define digitalPinToInterrupt(p)	(((p) <= 3 || (p) == 6 || (p) == 8) ? (p) : ((p) == 4 ? 5 : ((p) == 16 ? 4 : -1)))
#elif defined(__AVR_ATmega32U4__)
#define analogInputToDigitalPin(ch)	((ch) <= 10 ? 21 - (ch) : ((ch) == 11 ? 22 : -1))
#define digitalPinHasPWM(p)		((p) == 4 || (p) == 5 || (p) == 9 || (p) == 10 || (p) == 12 || (p) == 14 || (p) == 15)
#define digitalPinToInterrupt(p)	(((p) >= 5 && (p) <= 8) ? (p) - 5 : -1)
#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
#define analogInputToDigitalPin(ch)	((ch) <= 7 ? (ch) + 38 : -1)
#define digitalPinHasPWM(p)		(((p) >= 14 && (p) <= 16) || ((p) >= 24 && (p) <= 27) || (p) == 0 || (p) == 1)
#define digitalPinToInterrupt(p)	((p) <= 3 ? (p) : (((p) == 36 || (p) == 37) ? (p) - 32 : (((p) == 18 || (p) == 19) ? (p) - 12 : -1)))
#endif

#if defined(__AVR_AT90USB162__)
#define digitalPinToPortReg(p) (((p) <= 7) ? &PORTD : (((p) <= 15) ? &PORTB : &PORTC))
#define digitalPinToBit(p) \
	(((p) <= 7) ? (p) : (((p) <= 15) ? (p) - 8 : (((p) <= 19) ? 23 - (p) : 2)))
#define digitalPinToPCICR(p) \
	((((p) >= 8 && (p) <= 15) || ((p) >= 17 && (p) <= 20) || (p) == 5) ? &PCICR : NULL)
#define digitalPinToPCICRbit(p) (((p) >= 8 && (p) <= 15) ? 0 : 1)
#define digitalPinToPCIFR(p) \
	((((p) >= 8 && (p) <= 15) || ((p) >= 17 && (p) <= 20) || (p) == 5) ? &PCIFR : NULL)
#define digitalPinToPCIFRbit(p)	(((p) >= 8 && (p) <= 15) ? 0 : 1)
#define digitalPinToPCMSK(p) \
	(((p) >= 8 && (p) <= 15) ? &PCMSK0 : ((((p) >= 17 && (p) <= 20) || (p) == 5) ? &PCMSK1 : NULL))
#define digitalPinToPCMSKbit(p) \
	(((p) >= 8 && (p) <= 15) ? (p) - 8 : (((p) >= 17 && (p) <= 20) ? (p) - 17 : 4))

#elif defined(__AVR_ATmega32U4__)
#define digitalPinToPortReg(p) \
	(((p) <= 4) ? &PORTB : (((p) <= 8) ? &PORTD : (((p) <= 10) ? &PORTC : (((p) <= 12) ? &PORTD : \
	(((p) <= 15) ? &PORTB : (((p) <= 21) ? &PORTF : (((p) <= 23) ? &PORTD : &PORTE)))))))
#define digitalPinToBit(p) \
	(((p) <= 3) ? (p) : (((p) == 4) ? 7 : (((p) <= 8) ? (p) - 5 : (((p) <= 10) ? (p) - 3 : \
	(((p) <= 12) ? (p) - 5 : (((p) <= 15) ? (p) - 9 : (((p) <= 19) ? 23 - (p) : \
	(((p) <= 21) ? 21 - (p) : (((p) <= 23) ? (p) - 18 : 6)))))))))
#define digitalPinToPCICR(p)	((((p) >= 0 && (p) <= 4) || ((p) >= 13 && (p) <= 15)) ? &PCICR : NULL)
#define digitalPinToPCICRbit(p)	(0)
#define digitalPinToPCIFR(p)	((((p) >= 0 && (p) <= 4) || ((p) >= 13 && (p) <= 15)) ? &PCIFR : NULL)
#define digitalPinToPCIFRbit(p)	(0)
#define digitalPinToPCMSK(p)	((((p) >= 0 && (p) <= 4) || ((p) >= 13 && (p) <= 15)) ? &PCMSK0 : NULL)
#define digitalPinToPCMSKbit(p) \
	(((p) >= 0 && (p) <= 3) ? (p) : (((p) >= 13 && (p) <= 15) ? (p) - 9 : 7))

#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
#define digitalPinToPortReg(p) \
	(((p) >= 0 && (p) <= 7) ? &PORTD : (((p) >= 10 && (p) <= 17) ? &PORTC : \
	(((p) >= 20 && (p) <= 27) ? &PORTB : (((p) >= 28 && (p) <= 35) ? &PORTA : \
	(((p) >= 38 && (p) <= 45) ? &PORTF : &PORTE)))))
#define digitalPinToBit(p) \
	(((p) <= 7) ? (p) : (((p) <= 9) ? (p) - 8 : (((p) <= 17) ? (p) - 10 : \
	(((p) <= 19) ? (p) - 12 : (((p) <= 27) ? (p) - 20 : (((p) <= 35) ? (p) - 28 : \
	(((p) <= 37) ? (p) - 32 : (((p) <= 45) ? (p) - 38 : 2))))))))
#define digitalPinToPCICR(p)	(((p) >= 20 && (p) <= 27) ? &PCICR : NULL)
#define digitalPinToPCICRbit(p)	(0)
#define digitalPinToPCIFR(p)	(((p) >= 20 && (p) <= 27) ? &PCIFR : NULL)
#define digitalPinToPCIFRbit(p)	(0)
#define digitalPinToPCMSK(p)	(((p) >= 20 && (p) <= 27) ? &PCMSK0 : NULL)
#define digitalPinToPCMSKbit(p)	(((p) - 20) & 7)
#endif

#define NOT_ON_TIMER 0
static inline uint8_t digitalPinToTimer(uint8_t) __attribute__((always_inline, unused));
static inline uint8_t digitalPinToTimer(uint8_t pin)
{
	switch (pin) {
	#ifdef CORE_PWM0_PIN
	case CORE_PWM0_PIN: return 1;
	#endif
	#ifdef CORE_PWM1_PIN
	case CORE_PWM1_PIN: return 2;
	#endif
	#ifdef CORE_PWM2_PIN
	case CORE_PWM2_PIN: return 3;
	#endif
	#ifdef CORE_PWM3_PIN
	case CORE_PWM3_PIN: return 4;
	#endif
	#ifdef CORE_PWM4_PIN
	case CORE_PWM4_PIN: return 5;
	#endif
	#ifdef CORE_PWM5_PIN
	case CORE_PWM5_PIN: return 6;
	#endif
	#ifdef CORE_PWM6_PIN
	case CORE_PWM6_PIN: return 7;
	#endif
	#ifdef CORE_PWM7_PIN
	case CORE_PWM7_PIN: return 8;
	#endif
	#ifdef CORE_PWM8_PIN
	case CORE_PWM8_PIN: return 9;
	#endif
	default: return NOT_ON_TIMER;
	}
}

#define SERIAL_PORT_MONITOR             Serial
#define SERIAL_PORT_USBVIRTUAL          Serial
#define SERIAL_PORT_HARDWARE            Serial1
#define SERIAL_PORT_HARDWARE_OPEN       Serial1

#define SerialUSB			Serial

#endif
