/*
 * sensoray.h
 *
 *  Created on: Oct 11, 2012
 *      Author: Adam Barber
 *
 *  Contains definitions and function call prototypes for interacting with sensoray 526
 */

#ifndef SENSORAY_H_
#define SENSORAY_H_

//#include <stdlib.h>
//#include <stdio.h>
//#include <stdint.h>
//#include <unistd.h>
//#include <sys/mman.h>
//#include <sys/neutrino.h>
//#include <hw/inout.h>
#include "includes.h"

//Address Registers						//Write/Read(if nec)
#define 	SENSORAY_BASE_ADDR 	0x2C0
#define		TCR_ADDR			0x00 	//Timer Control Register
#define		WDC_ADDR			0x02	//Watchdog Timer Control Register
#define		DAC_ADDR			0x04	//DAC Control
#define		ADC_ADDR			0x06	//ADC Control
#define		ADD_ADDR			0x08	//DAC Data 							/ ADC Data
#define		DIO_ADDR			0x0A	//Digital IO control				/ Digital IO data
#define		IER_ADDR			0x0C	//Interrupt enable register
#define		ISR_ADDR			0x0E	//Interrupt status register			/ Interrupt status register
#define		MSC_ADDR			0x10	//Miscellaneous register			/ Miscellaneous register
//Counter addresses
#define		C0L_ADDR			0x12	//Counter 0 preload reg low word	/ Counter 0 data low word
#define		C0H_ADDR			0x14	//Counter 0 preload reg high word	/ Counter 0 data high word
#define		C0M_ADDR			0x16	//Counter 0 mode register
#define		C0C_ADDR			0x18	//Counter 0 control register		/ Counter 0 status register

#define		C1L_ADDR			0x1A	//Counter 1 preload reg low word	/ Counter 1 data low word
#define		C1H_ADDR			0x1C	//Counter 1 preload reg high word	/ Counter 1 data high word
#define		C1M_ADDR			0x1E	//Counter 1 mode register
#define		C1C_ADDR			0x20	//Counter 1 control register		/ Counter 1 status register

#define		C2L_ADDR			0x22	//Counter 2 preload reg low word	/ Counter 2 data low word
#define		C2H_ADDR			0x24	//Counter 2 preload reg high word	/ Counter 2 data high word
#define		C2M_ADDR			0x26	//Counter 2 mode register
#define		C2C_ADDR			0x28	//Counter 2 control register		/ Counter 2 status register

#define		C3L_ADDR			0x2A	//Counter 3 preload reg low word	/ Counter 3 data low word
#define		C3H_ADDR			0x2C	//Counter 3 preload reg high word	/ Counter 3 data high word
#define		C3M_ADDR			0x2E	//Counter 3 mode register
#define		C3C_ADDR			0x30	//Counter 3 control register		/ Counter 3 status register
//EEPROM Address
#define		EED_ADDR			0x32	//EEPROM data						/ EEPROM data
#define		EEC_ADDR			0x34	//EEPROM interface command			/ Signature


//Write bits, use bitwise OR | to combine words to form a complete command
//DO these later
#define		EEPROM_READ			0x0005

//function prototypes
uintptr_t initialize_sensoray();
void initialize_encoder_1(uintptr_t iobase);
void initialize_encoder_2(uintptr_t iobase);
void initialize_encoder_3(uintptr_t iobase);
int initialize_digitalIO(uintptr_t iobase);
int initialize_analog_out(uintptr_t iobase);
int read_encoder_1(uintptr_t iobase);
int read_encoder_2(uintptr_t iobase);
int read_encoder_3(uintptr_t iobase);

//Variables
//DAC Calibration parameters
union DACCal {
	double cal;
	unsigned short eeprom_data[4];
} DACa0, DACb0, DACa1, DACb1, DACa2, DACb2, DACa3, DACb3;


#endif /* SENSORAY_H_ */
