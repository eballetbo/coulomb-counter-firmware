/**
 * \file
 *
 * \brief User board initialization template
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>

void board_init(void)
{
	/* This function is meant to contain board-specific initialization code
	 * for, e.g., the I/O pins. The initialization can rely on application-
	 * specific board configuration, found in conf_board.h.
	 */
	
	/* USART TX pin configuration */
	PORTC_DIR = PIN3_bm;
	PORTC_OUTSET = PIN3_bm;

	/* Current calibration pin default configuration (disabled) */
	ioport_set_pin_dir(CONF_PIN_ADC_CAL, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CONF_PIN_ADC_CAL, 0);

	/* Interrupt pin default configuration (high - active low)*/
	ioport_set_pin_dir(CONF_PIN_INT, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CONF_PIN_INT, 1);

	/* TEMPSEL pin default configuration (select VTEMP) */
	ioport_set_pin_dir(CONF_PIN_TEMPSEL, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CONF_PIN_TEMPSEL, 0);

	/* EEPROM write protect pin default configuration (write operation inhibited) */
	ioport_set_pin_dir(CONF_PIN_AT24_WP, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CONF_PIN_AT24_WP, 1);

	/* Configure TWIC - SCL output; SDA inputs */
	//ioport_configure_port_pin(&PORTC, PIN0_bm, IOPORT_INIT_HIGH | IOPORT_DIR_OUTPUT);
	//ioport_configure_port_pin(&PORTC, PIN1_bm, IOPORT_DIR_INPUT);

	/* Configure SPIC - MISO output; MOSI, SCK, SS inputs */
	ioport_configure_port_pin(&PORTC, PIN4_bm, IOPORT_DIR_INPUT);
	ioport_configure_port_pin(&PORTC, PIN5_bm, IOPORT_DIR_INPUT);
	ioport_configure_port_pin(&PORTC, PIN6_bm, IOPORT_INIT_HIGH | IOPORT_DIR_OUTPUT);
	ioport_configure_port_pin(&PORTC, PIN7_bm, IOPORT_DIR_INPUT);
}
