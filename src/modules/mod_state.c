/**
 * @file    mod_state.c
 * @brief   Handles different states/modes and commands of the e-puck.
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

#include <mod_state.h>
#include <mod_draw.h>
#include <mod_communication.h>
#include <mod_data.h>

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

#define CMD_PERIOD			100

/*===========================================================================*/
/* Module thread pointers.                                                   */
/*===========================================================================*/

static thread_t* ptr_process_cmd;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/


/**
 * @brief				Processes command and calls relevant module functions.
 *
 * @param[in] 	cmd 	Command. Possible commands listed in module constants.
 */
static void process_command(uint8_t cmd)
{
	cartesian_coord* pos;
	uint16_t length;
	switch(cmd) {
		case CMD_RESET:
//			palClearPad(GPIOD, GPIOD_LED1);
//			chThdSleepMilliseconds(1000);
//			palSetPad(GPIOD, GPIOD_LED1);
//			chprintf((BaseSequentialStream *)&SDU1, "RESET \r \n");
			draw_reset();
			data_free();
			break;
		case CMD_PAUSE:
			draw_set_init_length(100);
			break;
		case CMD_CONTINUE:
			break;
		case CMD_CALIBRATE:
			draw_reset();
			break;
		case CMD_GET_DATA:
			com_receive_data((BaseSequentialStream *)&SD3);
			break;
		case CMD_DRAW:
			pos = data_get_pos();
			length = data_get_length();
			for(uint16_t i = 0; i<length;++i) {
				draw_move(pos[i].x, pos[i].y);
			}

			break;
		case CMD_INTERACTIVE:
			draw_move(512, 0);
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
 */
static THD_WORKING_AREA(wa_process_cmd, 1024);
static THD_FUNCTION(thd_process_cmd, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while(1) {
		uint8_t cmd = com_receive_command((BaseSequentialStream *)&SD3);
		process_command(cmd);
		chThdSleepMilliseconds(CMD_PERIOD);
	}
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void create_thd_process_cmd(void)
{
	ptr_process_cmd = chThdCreateStatic(wa_process_cmd, sizeof(wa_process_cmd), NORMALPRIO, thd_process_cmd, NULL);
}



