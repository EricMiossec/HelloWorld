/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "calib.h"

#define UART1_TX 9
#define UART1_RX 10

static ShellCommand local_commands[] = {
	{"info", cmd_info},
	{"systime", cmd_systime},
	{"calib", cmd_calib},
	{NULL, NULL}
};

static const ShellConfig shell_cfg = {
(BaseChannel *)&SD1,
local_commands
};

static uint16_t measure = 0;
static WORKING_AREA(waI2CThread, 128);
static msg_t I2CThread(void *arg)
{
	while(1)
	{
		chThdSleepMilliseconds(1000);
		update_volt_measures();
	}
}

static uint32_t seconds_counter = 0;
/*
 * This is a periodic thread that does absolutely nothing except increasing
 * a seconds counter.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

	(void)arg;
	chRegSetThreadName("counter");
	while (TRUE){
		chThdSleepMilliseconds(1000);
		seconds_counter++;
	}
}

void GPIO_Configuration()
{
	/* Serial Driver Pin Config*/	
	palSetPadMode(GPIOA, UART1_TX, PAL_MODE_STM32_ALTERNATE_PUSHPULL);	
	palSetPadMode(GPIOA, UART1_RX, PAL_MODE_INPUT);
	
	// ADC driver input pin
	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
}

/*
 * Application entry point.
 */
int main(void) {
	
	/* Shell thread */
	Thread *shelltp = NULL;
	
	 /* UART Configuration Structure */
	SerialConfig sc;

	// Dummy ADC Configuration structure
	ADCConfig adc_conf;
	
	/*
	* System initializations.
	* - HAL initialization, this also initializes the configured device drivers
	*   and performs the board-specific initializations.
	* - Kernel initialization, the main() function becomes a thread and the
	*   RTOS is active.
	*/
	halInit();
	chSysInit();
	GPIO_Configuration();
	//ADC_Configuration();
	I2C_Configuration();
	
	 /* Start Serial Driver */
	sc.sc_speed = 115200;
	sc.sc_cr1 = 0;
	sc.sc_cr2 = USART_CR2_STOP1_BITS | USART_CR2_LINEN;
	sc.sc_cr3 = 0;
	sdStart(&SD1, &sc);
	
	/* Shell manager initialization. */
    shellInit();
	
	/*
	* Creates the example thread.
	*/
	chThdCreateStatic(waThread1, sizeof(waI2CThread), NORMALPRIO, I2CThread, NULL);
	
	/*
	* Normal main() thread activity, in this demo it does nothing except
	* sleeping in a loop and check the button state, when the button is
	* pressed the test procedure is launched with output on the serial
	* driver 1.
	*/
	while (TRUE) {
		if(!shelltp)
		{
			shelltp = shellCreate( &shell_cfg, THD_WA_SIZE(2048), NORMALPRIO);
		}
		else if(chThdTerminated(shelltp))
		{
			// Recovers memory of the previous shell.
			chThdRelease(shelltp);
			shelltp = NULL;
		}
  	}
	
}
