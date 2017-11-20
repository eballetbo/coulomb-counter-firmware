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

#ifndef SBS_MM_H_
#define SBS_MM_H_

#define SBS_MEMORY_MAP_SIZE			128	/* 128 bytes - 0x80 */
#define SBS_MEMORY_MAP_CSUM_MSB_bp	(SBS_MEMORY_MAP_SIZE - 2) /* 0x7e */
#define SBS_MEMORY_MAP_CSUM_LSB_bp	(SBS_MEMORY_MAP_SIZE - 1) /* 0x7f */
#define SBS_MEMORY_MAP_NVM_BASE		0x00
#define SBS_MEMORY_MAP_NVM_SIZE		0x70
#define SBS_MEMORY_MAP_RAM_BASE		0x70
#define SBS_MEMORY_MAP_RAM_SIZE		0x10

#define SBS_CHARGING_VOLTAGE		0x0a	/* Charging voltage, 2 bytes */
#define SBS_DESIGN_VOLTAGE			0x0c	/* Design voltage, 2 bytes */
#define SBS_FAST_CHARGING_CURRENT	0x0e	/* Fast charging current, 2 bytes */
#define SBS_MAX_LOW_TEMPERATURE		0x10	/* Max T, Low T, 2 bytes */
#define SBS_PACK_CAPACITY			0x12	/* Pack capacity, 2 bytes */
#define SBS_SERIAL_NUMBER			0x18	/* Serial number, 2 bytes */
#define SBS_MANUFACTURER_NAME		0x20	/* Manufacturer name, 16 bytes */
#define SBS_MODEL_NAME				0x30	/* Model name, 16 bytes */
#define SBS_DEVICE_CHEMISTRY		0x40	/* Device chemistry, 5 bytes */
#define SBS_VOLTAGE_NOW				0x70	/* Voltage now, 2 bytes */
#define SBS_CURRENT_NOW				0x72	/* Current now, 2 bytes */
#define SBS_BATTERY_STATUS			0x74	/* Battery Status, 2 bytes */
#define SBS_STATE_OF_CHARGE			0x76	/* State of Charge in percentage, 1 byte */
#define SBS_CYCLE_COUNT				0x77	/* Cycle count, 2 bytes */
#define SBS_FIRMWARE_VERSION		0x7d	/* Firmware version, 1 byte */

extern volatile uint8_t memory_map[SBS_MEMORY_MAP_SIZE];

int8_t mm_write(uint8_t address, const void* buf, uint8_t len);
void mm_read(uint8_t address, void* buf, uint8_t len);
void mm_erase_all(void);
void mm_init(void);
void mm_dump(void);

#endif /* SBS_MM_H_ */