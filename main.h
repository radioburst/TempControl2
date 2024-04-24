/************************************************************************/
/*                                                                      */
/*                      1-Wire Example					*/
/*                                                                      */
/*              Author: Peter Dannegger                                 */
/*                      danni@specs.de                                  */
/*                                                                      */
/************************************************************************/
#ifndef _main_h_
#define _main_h_

#ifndef F_CPU
	#define F_CPU		12000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
//#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>

enum ERRORSTATE{

	NOERROR = 0,
	RUNTIMEERROR = 1,
	MAINSENSORERROR = 2
};

#define uchar unsigned char
#define uint unsigned int
#define bit uchar
#define idata
#define code

#define W1_PIN	PD0
#define W1_IN	PIND
#define W1_OUT	PORTD
#define W1_DDR	DDRD

extern volatile uchar id1[9], id2[9];
extern volatile float fTist, fTist2;
extern volatile uint8_t uiMainSensorLastReading, uiBeerSensorLastReading;

//#include "1wire.h"
//#include "ds18b20.h"

#endif
