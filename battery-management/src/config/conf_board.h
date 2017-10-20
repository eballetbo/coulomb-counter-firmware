/**
 * \file
 *
 * \brief User board configuration template
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

#define F_CPU	32000000UL
#define CONF_BOARD_ENABLE_USARTC0

/* Board PINS */
#define CONF_PIN_ADC_CAL			IOPORT_CREATE_PIN(PORTB, 2)
#define CONF_PIN_INT				IOPORT_CREATE_PIN(PORTB, 3)
#define CONF_PIN_TEMPSEL			IOPORT_CREATE_PIN(PORTE, 2)
#define CONF_PIN_AT24_WP			IOPORT_CREATE_PIN(PORTE, 3)

#define CONF_SBS_I2C_EEPROM_ADDR		0x52
#define CONF_SBS_I2C_EEPROM_SPEED		50000	/* 50 kHz*/

/* Debug */
#ifdef DEBUG
#define pr_debug(...) printf( __VA_ARGS__ )
#else
#define pr_debug(...) do{ } while ( false )
#endif

#endif // CONF_BOARD_H
