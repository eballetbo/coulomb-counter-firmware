/**
 * \file
 *
 * \brief Smart Battery System driver - SPI
 *
 * This file defines a useful set of functions for the SPI interface
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

#include "sbs.h"
#include "sbs_spi.h"

volatile uint8_t addr;

/** 
 * \brief SPI service interrupt handler
 *
 * \note Declare values that change in interrupts as volatile
 */
ISR(SPIC_INT_vect) {
	/* Grab the address */
	addr = SPIC.DATA;
	/* If the master tries to read over the map size return a dummy byte */
	if (addr >= SBS_MEMORY_MAP_SIZE) {
		SPIC.DATA = SPI_DUMMY_BYTE;
	} else {
		/* Send the value to the master */
		SPIC.DATA = memory_map[addr];
	}
}

/** 
 * \brief
 */
void spi_slave_init(void)
{
	uint8_t dummy;

	/* Enable SPI peripheral clock */
	sysclk_enable_peripheral_clock(&SPIC);

	/* Configure SPI on PORTC */
	SPIC.CTRL = 0x40;          // spi slave, spi mode 0
	SPIC.INTCTRL = 0x3;        // assign high priority to SPIC interrupts

	/* Flush slave receive buffer */
	while(SPIC.STATUS & 0x80) {
		dummy = SPIC.DATA;   // flush spi receive buffer
	}

	dummy = SPI_DUMMY_BYTE;
	SPIC.DATA = dummy;
}