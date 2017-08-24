/**
 * \file
 *
 * \brief Coulomb Counter Application
 *
 */

/**
 * \mainpage Coulomb Counter Application doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <errno.h>
#include <string.h>
#include <util/delay.h>
#include "sbs.h"
#include "sbs_adc.h"
#include "sbs_mm.h"
#include "sbs_spi.h"

#define _VERSION_	"1"

/* Globals */
static bool is_in_test_mode = false;

/* Serial DEBUG configuration */
const static usart_serial_options_t USART_SERIAL_OPTIONS = {
	.baudrate = USART_SERIAL_BAUDRATE,
	.charlength = USART_SERIAL_CHAR_LENGTH,
	.paritytype = USART_SERIAL_PARITY,
	.stopbits = USART_SERIAL_STOP_BIT,
};

static void do_toggle_output_pins(void)
{
	printf("Toggle all output pins\n\r");
	ioport_toggle_pin_level(CONF_PIN_ADC_CAL);
	ioport_toggle_pin_level(CONF_PIN_INT);
	ioport_toggle_pin_level(CONF_PIN_TEMPSEL);
	ioport_toggle_pin_level(CONF_PIN_AT24_WP);
}

/**
 * \brief Enter to test mode
 *
 */
static void enter_test_mode(void)
{
	printf("\n\rWelcome to the test mode. Press\n\r");
	printf(" - key 'q' to exit from test mode\n\r");
	printf(" - key 'd' to dump the Memory Map contents\n\r");
	printf(" - key 'e' to erase Memory Map\n\r");
	printf(" - key 'p' to program default Memory Map\n\r");
	printf(" - key 't' to toggle output pins\n\r");

	while (is_in_test_mode) {
		if (usart_rx_is_complete(USART_SERIAL_DEBUG)) {
			char key = getchar();
			if (key == 'q') {
				is_in_test_mode = false;
				break;
			} else if (key == 'd') {
				mm_dump();
			} else if (key == 'p') {
				mm_init();
			} else if (key == 'e') {
				mm_erase_all();
			} else if (key == 't') {
				do_toggle_output_pins();
			}
		}
	}
}

int main(void)
{
	/* Modules initializations */
	sysclk_init();
	/* Board specific initialization */
	board_init();
	/* Initialize the PMIC */
	pmic_init();
	/* Startup the RTC and start counting from zero */
	rtc_init();
	/* Initialize serial debug interface using stdio library */
	stdio_serial_init(USART_SERIAL_DEBUG, &USART_SERIAL_OPTIONS);
	/* Initialize SPIC as slave */
	spi_slave_init();
	/* Initialize interrupt vectors */
	//irq_initialize_vectors();
	PMIC.CTRL = 0x04;
	/* Enable global interrupt */
	cpu_irq_enable();

	printf("\x0C\n\r-- TCL Battery Coulomb Counter --\n\r");
	printf("-- Version: %s --\n\r", _VERSION_);
	/* printf("-- Version: %s (build %s) --\n\r", _VERSION_, __TIMESTAMP__); */
	printf("\r\nPress 'd' to enter in test mode\n\r");

	/* Do the ADC current sense calibration process */
	do_current_sense_calibration();

	/* Program the default values if the memory is empty */
	mm_init();

	while(true) {
		if (usart_rx_is_complete(USART_SERIAL_DEBUG)) {
			char key = getchar();
			if (key == 'd') {
				is_in_test_mode = true;
				/* Loops until the user exits from test mode */
				enter_test_mode();
			}
		}

		/* Update memory map with current data */
		update_memory_map();

		_delay_ms(1000);
	}
}