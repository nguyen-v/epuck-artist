/**
 * @file    main.c
 * @brief   Main file. Handles interactions between modules.
 */

// C standard header files

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ChibiOS headers

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "chprintf.h"
#include <usbcfg.h>

// Module headers

#include <main.h>
#include <mod_communication.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

// List of possible commands sent from the computer

#define CMD_RESET			'R'
#define CMD_PAUSE			'P'
#define CMD_CONTINUE		'C'
#define CMD_CALIBRATE		'B'
#define CMD_GET_DATA		'G'
#define CMD_DRAW			'D'
#define CMD_INTERACTIVE		'I'

// Periods

#define PERIOD_CMD		1000

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief				Initalizes all modules.
 */
static void init_all(void)
{
	halInit();
	chSysInit();
	mpu_init();
	usb_start();
	com_serial_start();
}

/**
 * @brief				Processes command and calls relevant module functions.
 *
 * @param[in] 	cmd 	Command. Possible commands listed in module constants.
 */
static void process_command(uint8_t cmd)
{
	switch(cmd) {
		case CMD_RESET:
			palClearPad(GPIOD, GPIOD_LED1);
			chThdSleepMilliseconds(1000);
			palSetPad(GPIOD, GPIOD_LED1);
			break;
		case CMD_PAUSE:
			break;
		case CMD_CONTINUE:
			break;
		case CMD_CALIBRATE:
			break;
		case CMD_GET_DATA:
			com_receive_data((BaseSequentialStream *)&SD3);
			break;
		case CMD_DRAW:
			break;
		case CMD_INTERACTIVE:
			break;
		default:
			chprintf((BaseSequentialStream *)&SDU1, "Invalid command");
			break;
	}
}

/*===========================================================================*/
/* Module threads.                                                   		 */
/*===========================================================================*/

/**
 * @brief	Thread for reading command from serial (SD3).
 *
 * @note	A mutex is used to restrict access to the serial
 * 			by the thread when the e-puck is reading move data.
 */
//static THD_WORKING_AREA(wa_process_cmd, 256);
//static THD_FUNCTION(thd_process_cmd, arg)
//{
//	chRegSetThreadName(__FUNCTION__);
//	(void)arg;
//
//	while(1) {
//		uint8_t cmd = com_receive_command((BaseSequentialStream *)&SD3);
//		process_command(cmd);
//		chThdSleepMilliseconds(CMD_PERIOD);
//	}
//}

/*===========================================================================*/
/* Main function.                                                   		 */
/*===========================================================================*/

int main(void)
{
	init_all();
//	chThdCreateStatic(wa_process_cmd, sizeof(wa_process_cmd), NORMALPRIO, thd_process_cmd, NULL);
	while(1) {
		process_command(com_receive_command((BaseSequentialStream *)&SD3));
		chThdSleepMilliseconds(PERIOD_CMD);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
