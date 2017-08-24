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

#ifndef SBS_ADC_H_
#define SBS_ADC_H_

/* Number of samples for average */
#define CONFIG_ADC_AVERAGE_COUNT	25
/* Multiply ADC reading by this then divide by 1000 to get mV */
#define CONFIG_ADC_VOLTAGE_SCALE	3174L /* (GAIN * 1000 / (1 << 12)) * 1000 */
#define CONFIG_ADC_VOLTAGE_BATT		ADCCH_POS_PIN2
#define CONFIG_ADC_CURRENT_LOAD		ADCCH_POS_PIN0
#define CONFIG_ADC_CURRENT_BATT		ADCCH_POS_PIN1
#define CONFIG_ADC_CURRENT_ZERO		ADCCH_POS_PIN4
/* Multiply ADC reading by this then divide by 1000 to get mA */
#define CONFIG_ADC_CURRENT_SCALE	462L

uint16_t adc_get_voltage_now(void);
int16_t adc_get_current_now(void);
void do_current_sense_calibration(void);

#endif /* SBS_ADC_H_ */