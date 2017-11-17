/**
 * \file
 *
 * \brief Smart Battery System driver - ADC
 *
 * This file defines a useful set of functions for the ADC interface
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
#include <util/delay.h>
#include "sbs_adc.h"

typedef struct calib_isense {
	int16_t		load;
	int16_t		batt;
} calib_isense_t;

volatile calib_isense_t calib_isense_temp, *calib_isense = &calib_isense_temp;

/**
 * \brief Returns the battery voltage in mV
 */
uint16_t adc_get_voltage_now(void)
{
	uint16_t sample = 0;
	uint32_t uV;
	uint8_t count;
	/* ADC module configuration structure */
	struct adc_config adc_conf;
	/* ADC channel configuration structure */
	struct adc_channel_config adcch_conf;

	/*
	 * Configure the ADC module:
	 * - unsigned, 12-bit results
	 * - internal 1V voltage reference
	 * - 200 kHz maximum clock rate
	 * - manual conversion triggering
	 */
	adc_read_configuration(&ADCA, &adc_conf);
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12,
								  ADC_REF_BANDGAP);
	adc_set_clock_rate(&adc_conf, 200000UL);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_write_configuration(&ADCA, &adc_conf);

	/*
	 * Configure ADC channel:
	 * - Single ended mode
	 * - Input voltage on ADC0 pin (PA2 pin)
	 * - 1x gain
	 * - interrupt flag set on completed conversion
	 */
	adcch_read_configuration(&ADCA, ADC_CH0, &adcch_conf);
	adcch_set_input(&adcch_conf, CONFIG_ADC_VOLTAGE_BATT, ADCCH_NEG_NONE, 1);
	adcch_set_interrupt_mode(&adcch_conf, ADCCH_MODE_COMPLETE);
	adcch_disable_interrupt(&adcch_conf);
	adcch_write_configuration(&ADCA, ADC_CH0, &adcch_conf);

	/* Enable ADC */
	adc_enable(&ADCA);

	/* Do useful conversion */
	sample = 0;
	for (count = 0; count < CONFIG_ADC_AVERAGE_COUNT; count++) {
		adc_start_conversion(&ADCA, ADC_CH0);
		adc_wait_for_interrupt_flag(&ADCA, ADC_CH0);
		sample += adc_get_unsigned_result(&ADCA, ADC_CH0);
	}
	sample /= CONFIG_ADC_AVERAGE_COUNT;
	
	/* Disable ADC */
	adc_disable(&ADCA);

	/* Conversion sample to mV */
	uV = CONFIG_ADC_VOLTAGE_SCALE * sample;
	return (uint16_t)(uV/1000L);
}

/**
 * \brief Configure the ADC module to do current measurements
 * 
 * Configuration:
 * - signed, 12-bit results
 * - internal 1V voltage reference
 * - 200 kHz maximum clock rate
 * - manual conversion triggering
 */
static void adc_current_setup(void)
{
	/* ADC module configuration structure */
	struct adc_config adc_conf;

	adc_read_configuration(&ADCA, &adc_conf);
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_ON, ADC_RES_12,
								  ADC_REF_BANDGAP);
	adc_set_clock_rate(&adc_conf, 200000UL);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_write_configuration(&ADCA, &adc_conf);
}

/**
 * \brief Configure the ADC channel to do current measurements
 *
 * Configuration:
 * - Differential mode
 * - Input voltage on ADC0 pins (pos and neg pins)
 * - 16x gain
 * - interrupt flag set on completed conversion
 *
 * \param pos Positive input signal.
 * \param neg Negative input signal:
 */
static void adcch_current_setup(enum adcch_positive_input pos, enum adcch_negative_input neg)
{
	/* ADC channel configuration structure */
	struct adc_channel_config adcch_conf;

	adcch_read_configuration(&ADCA, ADC_CH0, &adcch_conf);
	adcch_set_input(&adcch_conf, pos, neg, 16);
	adcch_set_interrupt_mode(&adcch_conf, ADCCH_MODE_COMPLETE);
	adcch_disable_interrupt(&adcch_conf);
	adcch_write_configuration(&ADCA, ADC_CH0, &adcch_conf);
}

/**
 * \brief Calculate the current sense offsets
 */
void do_current_sense_calibration(void)
{
	uint8_t i;

	/* To do this we should enable the CAL signal to ground the both inputs */
	ioport_set_pin_level(CONF_PIN_ADC_CAL, 1);
	/* Wait a moment to discharge filter capacitors */
	_delay_ms(1000);

	/* Configure the ADC module to do current measurements */
	adc_current_setup();
	/* Configure the ADC channels to read load current */
	adcch_current_setup(ADCCH_POS_PIN0, ADCCH_POS_PIN4);
	/* Enable ADC */
	adc_enable(&ADCA);
	/* Do useful conversion */
	calib_isense->load = 0;
	for (i = 0; i < CONFIG_ADC_AVERAGE_COUNT; i++) {
		adc_start_conversion(&ADCA, ADC_CH0);
		adc_wait_for_interrupt_flag(&ADCA, ADC_CH0);
		calib_isense->load += adc_get_signed_result(&ADCA, ADC_CH0);
	}
	calib_isense->load /= CONFIG_ADC_AVERAGE_COUNT;

	/* Configure the ADC channels to read battery current */
	adcch_current_setup(ADCCH_POS_PIN1, ADCCH_POS_PIN4);
	/* Do useful conversion */
	calib_isense->batt = 0;
	for (i = 0; i < CONFIG_ADC_AVERAGE_COUNT; i++) {
		adc_start_conversion(&ADCA, ADC_CH0);
		adc_wait_for_interrupt_flag(&ADCA, ADC_CH0);
		calib_isense->batt += adc_get_signed_result(&ADCA, ADC_CH0);
	}
	calib_isense->batt /= CONFIG_ADC_AVERAGE_COUNT;

	/* Disable ADC */
	adc_disable(&ADCA);
	/* Disable CAL signal */
	ioport_set_pin_level(CONF_PIN_ADC_CAL, 0);
}

/**
 * \brief Returns the battery current in mA
 */
int16_t adc_get_current_now(void)
{
	uint8_t count;
	/* Used to calculate the uA */
	int32_t uA;
	/* To store load current sense and battery current sense values */
	int16_t load_result, batt_result;
	
	/* Configure the ADC module to do current measurements */
	adc_current_setup();
	
	/* Configure the ADC channels to read load current */
	adcch_current_setup(CONFIG_ADC_CURRENT_LOAD, CONFIG_ADC_CURRENT_ZERO);

	/* Enable ADC */
	adc_enable(&ADCA);

	/* Do useful conversion */
	load_result = 0;
	for (count = 0; count < CONFIG_ADC_AVERAGE_COUNT; count++) {
		adc_start_conversion(&ADCA, ADC_CH0);
		adc_wait_for_interrupt_flag(&ADCA, ADC_CH0);
		load_result += adc_get_signed_result(&ADCA, ADC_CH0);
	}
	load_result /= CONFIG_ADC_AVERAGE_COUNT;
	load_result -= calib_isense->load;

	/* Configure the ADC channels to read battery current */
	adcch_current_setup(CONFIG_ADC_CURRENT_BATT, CONFIG_ADC_CURRENT_ZERO);

	/* Do useful conversion */
	batt_result = 0;
	for (count = 0; count < CONFIG_ADC_AVERAGE_COUNT; count++) {
		adc_start_conversion(&ADCA, ADC_CH0);
		adc_wait_for_interrupt_flag(&ADCA, ADC_CH0);
		batt_result += adc_get_signed_result(&ADCA, ADC_CH0);
	}
	batt_result /= CONFIG_ADC_AVERAGE_COUNT;
	batt_result -= calib_isense->batt;

	/* Disable ADC */
	adc_disable(&ADCA);

	uA = CONFIG_ADC_CURRENT_SCALE * (load_result - batt_result);

	return (int16_t)(uA/1000L);
}