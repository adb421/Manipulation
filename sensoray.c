/*
 * sensoray.c
 *
 *  Created on: Oct 11, 2012
 *      Author: Adam
 *
 *  Some implementation functions for the sensoray 526
 */

#include "sensoray.h"

//Initialize the sensoray and return the pointer to the base, requires ThreadCtl(_NTO_TCTL_IO,0); called beforehand
uintptr_t initialize_sensoray() {
	uintptr_t iobase;
	//Map the registers to iobase
	iobase = mmap_device_io(54, SENSORAY_BASE_ADDR);
	if(iobase == MAP_DEVICE_FAILED) {
		printf("mmap_device_io fail\n");
		return NULL;
	}
	else
		return iobase;
}

void initialize_encoder_1(uintptr_t iobase) {
	//First write to mode control for counter 0
	//Write to preload 0, latch on read, quadrature count direction control, quadrature clock control
	out16(iobase + C0M_ADDR, 0x0480);
	//Reset counter, just in case.
	out16(iobase + C0C_ADDR, 0x8000);

	//Reset count
	current_position_RH14(iobase, 1);
}
void initialize_encoder_2(uintptr_t iobase) {
	//First write to mode control for counter 1
	//Write to preload 0, latch on read, quadrature count direction control, quadrature clock control
	out16(iobase + C1M_ADDR, 0x0480);
	//Reset counter, just in case.
	out16(iobase + C1C_ADDR, 0x8000);

	//Reset count
	current_position_RH11(iobase, 1);
}
void initialize_encoder_3(uintptr_t iobase){
	//First write to mode control for counter 0
	//Write to preload 0, latch on read, quadrature count direction control, quadrature clock control
	//Write to preload 0, latch on read, quadrature count direction control, quadrature clock control
	out16(iobase + C2M_ADDR, 0x0480);
	//Reset counter, just in case.
	out16(iobase + C2C_ADDR, 0x8000);

	//Reset count
	current_position_RH8(iobase, 1);
}

int read_encoder_1(uintptr_t iobase) {
	//Read low word first
	unsigned short low = in16(iobase+C0L_ADDR);
	unsigned short high = in16(iobase+C0H_ADDR);
	//Combine them
	uint32_t data = (int)(high & 0x00FF);
	data = (data << 16) | (low & 0xFFFF);
	return (int) data;
}

int read_encoder_2(uintptr_t iobase) {
	//Read low word first
	unsigned short low = in16(iobase+C1L_ADDR);
	unsigned short high = in16(iobase+C1H_ADDR);
	//Combine them
	uint32_t data = (int)(high & 0x00FF);
	data = (data << 16) | (low & 0xFFFF);
	return (int) data;
}

int read_encoder_3(uintptr_t iobase) {
	//Read low word first
	unsigned short low = in16(iobase+C2L_ADDR);
	unsigned short high = in16(iobase+C2H_ADDR);
	//Combine them
	uint32_t data = (int)(high & 0x00FF);
	data = (data << 16) | (low & 0xFFFF);

	return (int) data;
}
//Isn't really working
//int initialize_analog_out(uintptr_t iobase) {
//	//Initialize DAC calibration parameters.
//	//Two byte variables to hold the 8 bytes of the doubles
//	uint16_t temp48, temp32, temp16, temp0;
//	//Read DACa0
//	//a0 from low to high 0x00 - 0x03
//	out16(iobase + EEC_ADDR, EEPROM_READ | (0x0000 << 3));
//	temp0 = in16(iobase + EED_ADDR);
//	out16(iobase + EEC_ADDR, EEPROM_READ | (0x0001 << 3));
//	temp16 = in16(iobase + EED_ADDR);
//	out16(iobase + EEC_ADDR, EEPROM_READ | (0x0002 << 3));
//	temp32 = in16(iobase + EED_ADDR);
//	out16(iobase + EEC_ADDR, EEPROM_READ | (0x0003 << 3));
//	temp48 = in16(iobase + EED_ADDR);
////	DACa0 = (double)(((uint64_t)temp48 << 48) | ((uint64_t)temp32 << 32) | ((uint64_t)temp16 << 16) | (uint64_t)temp0);
//	DACa0.eeprom_data[3] = temp0;
//	DACa0.eeprom_data[2] = temp16;
//	DACa0.eeprom_data[1] = temp32;
//	DACa0.eeprom_data[0] = temp48;
//	printf("0-15: %x\n", temp0);
//	printf("16-31: %x\n", temp16);
//	printf("32-47: %x\n", temp32);
//	printf("48-63: %x\n", temp48);
//	printf("DACa0 = %f", DACa0.cal);//, (uint64_t)DACa0);
//	return 0;
//}
