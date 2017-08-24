/**
 * \file
 *
 * \brief Smart Battery System driver
 *
 * This file defines a useful set of functions for the SBS
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
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sbs.h"
#include "sbs_adc.h"
#include "sbs_mm.h"

/* Battery Status bits */
#define DISCHARGING			0x0040
#define FULLY_CHARGED		0x0020
#define FULLY_DISCHARGED	0x0010
#define CHARGING			0x0000

/* Measure the in-and-out-flowing energy */
static int32_t current_acc = 0;
static int32_t millicoulombs = 0;
/* Time for a complete discharge cycle */
static uint32_t time_to_empty = 0;
/* Time for a complete charge cycle */
static uint32_t time_to_full = 0;

/* Flag that indicates that a complete battery charged cycle was done */
static bool has_fully_charged = false;

/*
 * Flag to indicate that the current battery state is not known so we do
 * a voltage estimation for capacity.
 */
static bool voltage_estimation = true;

/**
 * \brief Update the voltage value in memory map.
 */
void update_voltage_now(void)
{
	uint16_t voltage;

	voltage = adc_get_voltage_now();
	
	mm_write(SBS_VOLTAGE_NOW, &voltage, sizeof(uint16_t));
}

/**
 * \brief Update the current value in memory map.
 */
void update_current_now(void)
{
	int16_t current;

	current = adc_get_current_now();

	mm_write(SBS_CURRENT_NOW, &current, sizeof(int16_t));
}

/**
 * \brief Update the cycle count value. Increment by one.
 */
void update_cycle_count(void)
{
	uint16_t cycles;

	cycles = read_cycle_count();

	mm_write(SBS_CYCLE_COUNT, &cycles, sizeof(uint16_t));
}

/**
 * \brief Returns the voltage value measurement.
 */
uint16_t read_voltage_now(void)
{
	uint16_t voltage;

	mm_read(SBS_VOLTAGE_NOW, &voltage, sizeof(uint16_t));
	
	return voltage;
}

/**
 * \brief Returns the current value measurement.
 */
int16_t read_current_now(void)
{
	uint16_t current;

	mm_read(SBS_CURRENT_NOW, &current, sizeof(int16_t));

	return (int16_t)current;
}

/**
 * \brief Returns the cycle counter value.
 */
uint16_t read_cycle_count(void)
{
	uint16_t cycles;

	mm_read(SBS_CYCLE_COUNT, &cycles, sizeof(uint16_t));

	return cycles;
}

/**
 * \brief Returns true when battery is fully charged.
 * 
 * When the battery is about to reach it's full charge, then the current drawn
 * by the battery from the charger drops to as low as 3% of the rated battery
 * capacity. For safety we will set a battery fully charged when reaches the 85%
 * of the design voltage.
 *
 * E.g: The design voltage of a pack is 4.2 and the battery capacity is
 * 5200mAh. So when the battery will be fully charged, the current drawn by the
 * battery will be reached as nearly 2% of 5200mA (104mA) and voltage should
 * greater than 4.03V
 *
 */
static bool is_fully_charged(void)
{
	#define FULLY_CHARGED_THRESHOLD	5
	static int threshold = 0;
	uint16_t voltage;
	int16_t current;
	int32_t vchg, ichg;

	mm_read(SBS_CHARGING_VOLTAGE, &voltage, sizeof(uint16_t));
	vchg = voltage * 95L / 100L;

	mm_read(SBS_PACK_CAPACITY, &current, sizeof(int16_t));
	ichg = current * 2L / 100L;

	mm_read(SBS_VOLTAGE_NOW, &voltage, sizeof(uint16_t));
	mm_read(SBS_CURRENT_NOW, &current, sizeof(int16_t));

	if ((voltage > vchg) && (abs(current) < ichg)) {
		if (threshold > FULLY_CHARGED_THRESHOLD)
			return true;
		else
			threshold++;
	} else {
		threshold = 0;
	}

	return false;
}

/**
 * \brief Returns true when battery is fully discharged
 *
 * Device should cut off when a lithium-ion battery reaches 3V/cell on
 * discharge. At this point the battery has about 5 percent capacity
 * left. This voltage threshold is to preserve some energy for housekeeping,
 * as well as to reduce battery stress and allow for some self-discharge if
 * the battery is not immediately recharged. This grace period in empty
 * state can last several months until self-discharge lowers the voltage of
 * Li-ion to about 2.5V/cell, at which point the protection circuit opens
 * and most packs become unserviceable with a regular charger.
 *
 * + info: http://batteryuniversity.com/learn/article/premature_voltage_cut_off
 */

/**
 * \brief Cut off voltage
 *
 * Below this limit the board is not working.
 */
#define CUT_OFF_VOLTAGE		3500L

/**
 * \brief Return true if battery is fully charged
 */
static bool is_fully_discharged(void)
{
	uint16_t voltage;

	mm_read(SBS_VOLTAGE_NOW, &voltage, sizeof(uint16_t));

	if (voltage < CUT_OFF_VOLTAGE)
		return true;
	
	return false;
}

/**
 * \brief Update the estimated remaining capacity.
 *
 * If we are uncalibrated and have not yet seen a full discharge or charge
 * cycle, we simply estimate the remaining capacity based on the battery
 * voltage. Otherwise the remaining capacity is calculated based on the
 * measured charge.
 */
void update_remaining_capacity(void)
{
	uint8_t state, rem_capacity;
	uint16_t voltage, vchg;

	mm_read(SBS_BATTERY_STATUS, &state, sizeof(uint8_t));

	if (state == FULLY_CHARGED) {
		rem_capacity = 100;
	} else if (state == FULLY_DISCHARGED) {
		rem_capacity = 0;
	} else { /* At this point battery is charging or discharging */
		if (voltage_estimation) {
			mm_read(SBS_VOLTAGE_NOW, &voltage, sizeof(uint16_t));
			mm_read(SBS_CHARGING_VOLTAGE, &vchg, sizeof(uint16_t));
			vchg -= 125L;	/* for safety, max. charge voltage - 125 mV */

			/* Linear approximation */
			rem_capacity = ((voltage - CUT_OFF_VOLTAGE) * 100L / (vchg - CUT_OFF_VOLTAGE));
			/* Handle fully charged and fully discharged cases */
			if (voltage > vchg)
				rem_capacity = 100;
			else if (voltage < CUT_OFF_VOLTAGE)
				rem_capacity = 0;
			/* At this point, check if rem_capacity is a valid value */
			if (rem_capacity > 100) {
				printf("WARN: Remaining capacity out of range: %d. Setting to 100\r\n", rem_capacity);
				rem_capacity = 100;
			} else if (rem_capacity < 0) {
				printf("WARN: Remaining capacity out of range: %d. Setting to 0\r\n", rem_capacity);
				rem_capacity = 0;
			}
			printf("INFO: Remaining capacity: %d (voltage estimated)\r\n", rem_capacity);
		} else {
			if (current_acc > millicoulombs)
				rem_capacity = 100;
			else if (current_acc < 0) {
				rem_capacity = 0;
			} else {
				rem_capacity = current_acc * 100L / millicoulombs;
			}

			if (rem_capacity > 100)
				rem_capacity = 100;

			printf("INFO: Remaining capacity: %d (accumulated current: %ld)\r\n", rem_capacity, current_acc);
		}
	}

	mm_write(SBS_STATE_OF_CHARGE, &rem_capacity, sizeof(uint8_t));
}

/**
 * \brief Update the Smart Battery's status word which contains Status bit flags.
 *
 * Status register bit mapped as follows:
 *   0x0040 - DISCHARGING
 *   0x0020 - FULLY_CHARGED
 *   0x0010 - FULLY_DISCHARGED
 *   0x0000 - CHARGING
 */
void update_battery_status(void)
{
	uint16_t status, status_old, capacity;
	int16_t current;

	mm_read(SBS_BATTERY_STATUS, &status_old, sizeof(uint16_t));
	mm_read(SBS_CURRENT_NOW, &current, sizeof(int16_t));
	mm_read(SBS_PACK_CAPACITY, &capacity, sizeof(uint16_t));

	if (is_fully_charged()) {
		status = FULLY_CHARGED;
		/* TODO: re-calc coulombs */
		millicoulombs = (capacity * 70L / 100L) * 3600L;
		current_acc = millicoulombs;
		printf("Total Battery Energy: %ld\r\n", millicoulombs);
		voltage_estimation = false;
	} else if (is_fully_discharged()) {
		status = FULLY_DISCHARGED;
		/* TODO: re-calc coulombs */
		current_acc = 0;
	} else if (current > 0) {
		status = DISCHARGING;
		current_acc -= current;
		if (current_acc < 0)
			current_acc = 0;
		printf("INFO: Energy counter: %ld\r\n", current_acc);
	} else {
		status = CHARGING;
		current_acc -= current;	/* negative number */
		if (current_acc > millicoulombs)
			current_acc = millicoulombs;
		printf("INFO: Energy counter: %ld\r\n", current_acc);
	}

	if (status_old == FULLY_DISCHARGED)
		printf("INFO: FULLY_DISCHARGED\r\n");
	else if (status_old == FULLY_CHARGED)
		printf("INFO: FULLY_CHARGED\r\n");
	else if (status_old == CHARGING)
		printf("INFO: CHARGING\r\n");
	else if (status_old == DISCHARGING)
		printf("INFO: DISCHARGING\r\n");

	/*
	 * Update the status if changed. State handling
	 */
	if (status_old != status) {
		mm_write(SBS_BATTERY_STATUS, &status, sizeof(uint16_t));
		
		if ((status_old == FULLY_CHARGED) && (status == DISCHARGING)) {
			/* 
			 * FULLY_CHARGED -> DISCHARGING
			 */
			printf("INFO: FULLY_CHARGED -> DISCHARGING\r\n");	
			/* Start up the RTC again and start counting from 0 the discharge time */
			rtc_init();
		} else if ((status_old == DISCHARGING) && (status == FULLY_DISCHARGED)) {
			/*
			 * DISCHARGING -> FULLY_DISCHARGED
			 */
			printf("INFO: DISCHARGING -> FULLY_DISCHARGED\r\n");
			/* Store the time to reach the fully discharged state */
			time_to_empty = rtc_get_time();
		} else if ((status_old == FULLY_DISCHARGED) && (status == CHARGING)) {
			/*
			 * FULLY_DISCHARGED -> CHARGING
			 */
			printf("INFO: FULLY_DISCHARGED -> CHARGING\r\n");
			/* Start up the RTC again and start counting from 0 the charge time */
			rtc_init();
		} else if ((status_old == CHARGING) && (status == FULLY_CHARGED)) {
			/*
			 * CHARGING -> FULLY_CHARGED
			 */
			printf("INFO: CHARGING -> FULLY_CHARGED\r\n");
			time_to_full = rtc_get_time(); /* Store the time to reach the fully charged state */
		} else if ((status_old == CHARGING) && (status == DISCHARGING)) {
			/*
			 * CHARGING -> DISCHARGING
			 */
			printf("INFO: CHARGING -> DISCHARGING\r\n");
		} else if ((status_old == DISCHARGING) && (status == CHARGING)) {
			/*
			 * DISCHARGING -> CHARGING
			 */
			printf("INFO: DISCHARGING -> CHARGING\r\n");
		} else {
			printf("WARN: Transition %02x -> %02x!\r\n", status_old, status);
			printf("WARN: Maybe battery is not connected?\r\n");
		}
	}
}

/**
 * \brief Fill memory map with random data.
 *
 * This function is intended to use for test purposes
 */
void fill_mm_with_random_data(void)
{
	uint8_t addr;
	uint16_t csum = 0;

	/* Fill data with random data*/
	for (addr = 0; addr < SBS_MEMORY_MAP_SIZE - 2; addr++) {
		memory_map[addr] = rand();
		csum +=  memory_map[addr];
	}

	/* Store the checksum */
	memory_map[SBS_MEMORY_MAP_CSUM_MSB_bp] = MSB(csum);
	memory_map[SBS_MEMORY_MAP_CSUM_LSB_bp] = LSB(csum);
}

/**
 * \brief Update memory map
 */
void update_memory_map(void)
{
	uint8_t addr;
	uint16_t csum = 0;

	update_voltage_now();
	update_current_now();
	update_battery_status();
	update_remaining_capacity();
	update_cycle_count();

	/* Recalculate the checksum of current memory map */
	for (addr = 0; addr < SBS_MEMORY_MAP_CSUM_MSB_bp; addr++) {
		csum +=  memory_map[addr];
	}

	/* Store the checksum for data integrity */
	memory_map[SBS_MEMORY_MAP_CSUM_MSB_bp] = MSB(csum);
	memory_map[SBS_MEMORY_MAP_CSUM_LSB_bp] = LSB(csum);

	printf("INFO: Voltage: %4d mV (0x%04x)\r\n", read_voltage_now(), read_voltage_now());
	printf("INFO: Current: %4d mA (0x%04x)\r\n", read_current_now(), read_current_now());
	printf("INFO: Timestamp: %ld\n\r", rtc_get_time());		
}