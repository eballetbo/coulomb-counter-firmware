/**
 * \file
 *
 * \brief Smart Battery System driver - Memory Map Management
 *
 * This file defines a useful set of functions for the memory management map interface
 *
 * Copyright (c) 2015 Toby Churchill Ltd.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include <asf.h>
#include <string.h>
#include "sbs_mm.h"
#include "errno.h"

/**
 * \brief Smart Battery Memory Map
 *
 *  +---------------+ 0x00
 *  |      OTP      |
 *  |---------------| 0x50
 *  |      NVM      |
 *  |---------------| 0x70
 *  |      RAM      |
 *  +---------------+ 0x80
 *
 *	0x0a - 0x0b : Charging voltage - 4200 mV (0x1068)
 *	0x0c - 0x0d : Design voltage - 3700 mV (0xE74)
 *	0x0e - 0x0f : Fast-Charging Current - 500 mA (0x1F4)
 *	0x10 - 0x11 : Max T, Low T - 60 degree / 0 degree
 *	0x12 - 0x13 : Pack Capacity  - 6900 mAh (0x1AF4)
 *	0x18 - 0x19 : Serial number
 *	0x20 - 0x2f : Manufacturer name
 *	0x30 - 0x3f : Model name
 *	0x40 - 0x44 : Device Chemistry
 *
 *	0x50 - 0x51 : Cycle count
 *
 *	0x70 - 0x71 : Voltage now
 *	0x72 - 0x73 : Current now
 *	0x74 - 0x75 : Battery status
 *  0x76        : State of charge
 *  0x7d        : Firmware version
 *  0x7e - 0x7f : Data Checksum
 */
volatile uint8_t memory_map[SBS_MEMORY_MAP_SIZE] = {
/*			 00    01    02    03    04    05    06    07    08    09    0a    0b    0c    0d    0e    0f */
/* 0000 */	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x10, 0x74, 0x0e, 0xf4, 0x01,
/* 0010 */	 60 , 0x00, 0xf4, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/* 0020 */	 'T',  'o',  'b',  'y',  ' ',  'C',  'h',  'u',  'r',  'c',  'h',  'i',  'l',  'l', 0x00, 0x00,
/* 0030 */	 '1',  '8',  '6',  '5',  '5',  ' ',  '1',  's',  '2',  'p', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/* 0040 */	 'L',  'I',  'O',  'N', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/* 0050 */	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/* 0060 */	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/* 0070 */	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

volatile uint8_t memory_map_default[SBS_MEMORY_MAP_SIZE] = {
	/*			 00    01    02    03    04    05    06    07    08    09    0a    0b    0c    0d    0e    0f */
	/* 0000 */	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x10, 0x74, 0x0e, 0xf4, 0x01,
	/* 0010 */	 60 , 0x00, 0xf4, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0020 */	 'T',  'o',  'b',  'y',  ' ',  'C',  'h',  'u',  'r',  'c',  'h',  'i',  'l',  'l', 0x00, 0x00,
	/* 0030 */	 '1',  '8',  '6',  '5',  '5',  ' ',  '1',  's',  '2',  'p', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0040 */	 'L',  'I',  'O',  'N', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0050 */	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0060 */	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0070 */	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


/**
 * \brief Write one byte to NVM memory map region.
 *
 * This function writes one byte to NVM memory map region.
 *
 * \param  address    Address (max SBS_MEMORY_MAP_NVM_SIZE)
 * \param  value      Byte value to write to NVM.
 */
static void mm_nvm_write_byte(uint8_t address, uint8_t value)
{
	/* TODO: Handle different NVM types */
	nvm_eeprom_write_byte(address, value);
}

/**
 * \brief Read one byte from NVM memory map region.
 *
 * This function reads one byte from NVM memory map region.
 *
 * \param  addr       Address, between 0 and SBS_MEMORY_MAP_NVM_SIZE
 *
 * \return  Byte value read from NVM.
 */
static uint8_t mm_nvm_read_byte(uint8_t address)
{
	/* TODO: Handle different NVM types */
	return nvm_eeprom_read_byte(address);
}

/**
 * \brief Write buffer to memory map
 *
 * \param address   the address to where to write
 * \param buf       pointer to the data
 * \param len       the number of bytes to write
 */
int8_t mm_write(uint8_t address, const void* buf, uint8_t len)
{
	uint8_t i, data;
	
	/* TODO: Check write out of memory map */
	
	for (i = 0; i < len; i++) {
		if (address < SBS_MEMORY_MAP_NVM_SIZE) {
			/* NVM write */
			mm_nvm_write_byte(address + i, ((uint8_t *)buf)[i]);
			/* Read back and update memory map (cached) */
			data = mm_nvm_read_byte(address + i);
			/* Verify that data was written */
			if (data != ((uint8_t *)buf)[i]) {
				printf("ERROR: Failed to verify data written to NVM (0x%02x != 0x%02x)\r\n", data, ((uint8_t *)buf)[i]);
				return -EINVAL;
			}
		}
		/* Update value in cached Memory Map */
		/* printf("DEBUG: Writing at address 0x%02x value 0x%02x\r\n", address + i, ((uint8_t *)buf)[i]); */
		memory_map[address + i] = ((uint8_t *)buf)[i];
	}

	return 0;
}

/**
 * \brief Read buffer from memory map
 *
 * \param address   the address to where to read
 * \param buf       pointer to the data
 * \param len       the number of bytes to read
 */
void mm_read(uint8_t address, void* buf, uint8_t len)
{
	memcpy(buf, (void*)(&memory_map[address]), len );
}

/** 
 * \brief Erase all the memory map
 */
void mm_erase_all(void)
{
	uint8_t addr, value = 0xff;
	
	printf("Erasing all memory map ... ");
	for (addr = 0; addr < SBS_MEMORY_MAP_SIZE; addr++)
		mm_write(addr, &value, 1);
	printf("done\r\n");
}

/** 
 * \brief Program default values.
 */
void mm_init(void)
{
	uint8_t addr, data;

	printf("Programming default values ... ");
	for (addr = 0; addr < SBS_MEMORY_MAP_SIZE; addr++) {
		data = memory_map_default[addr];
		mm_write(addr, &data, 1);
	}
	printf("done\r\n");
}

/**
 * \brief Dump memory map
 */
void mm_dump(void)
{
	uint8_t i, value;

	printf("Memory Map:\r\n");
	for (i = 0; i < SBS_MEMORY_MAP_SIZE; i++) {
		mm_read(i, &value, 1);
		printf("{ 0x%02x, 0x%02x }, ", i, value);
	}
	printf("\r\n");
}

#ifdef AT24_EEPROM
static void nvm_at24_twi_init(void)
{
	twi_master_options_t opt = {
		.speed = CONF_SBS_I2C_EEPROM_SPEED,
		.chip  = CONF_SBS_I2C_EEPROM_ADDR,
	};

	twi_master_setup(&TWIC, &opt);
}

static void nvm_at24_program_default_values(void)
{
	const uint8_t test_pattern[] = { 0x55, 0xA5, 0x5A, 0x77, 0x99 };
	uint8_t data_received[10];
	uint8_t i;
	twi_package_t packet_write = {
		.addr         = { 0, 0, 0 },              // TWI slave memory address data
		.addr_length  = sizeof (uint16_t),        // TWI slave memory address data size
		.chip         = CONF_SBS_I2C_EEPROM_ADDR, // TWI slave bus address
		.buffer       = (void *)test_pattern,     // transfer data source buffer
		.length       = sizeof(test_pattern)      // transfer data size (bytes)
	};
	twi_package_t packet_read = {
		.addr         = { 0, 0, 0 },               // TWI slave memory address data
		.addr_length  = sizeof (uint16_t),         // TWI slave memory address data size
		.chip         = CONF_SBS_I2C_EEPROM_ADDR,  // TWI slave bus address
		.buffer       = data_received,             // transfer data destination buffer
		.length       = 10                         // transfer data size (bytes)
	};

	/* Disable AT24 EEPROM write protect */
	ioport_set_pin_level(CONF_PIN_AT24_WP, 0);

	nvm_at24_twi_init();

	//while (twi_master_write(&TWIC, &packet_write) != TWI_SUCCESS);
	//twi_master_write(&TWIC, &packet_write);

	_delay_ms(500);

	// Perform a multi-byte read access then check the result.
	twi_master_read(&TWIC, &packet_read);

	//if(twi_master_read(&TWIC, &packet_read) == TWI_SUCCESS){
	//Check read content
	if (data_received[0] == 0x55) {
		printf("Read data from EEPROM: ");
		for (i = 0; i < 10; i++)
		printf("0x%02x ", data_received[i]);
		printf("\r\n");
		} else {
		printf("Failed to verify EEPROM data");
	}
	//}

	/* Enable AT24 EEPROM write protect*/
	ioport_set_pin_level(CONF_PIN_AT24_WP, 1);
}
#endif