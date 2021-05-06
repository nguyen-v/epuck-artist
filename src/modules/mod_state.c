/**
 * @file    mod_state.c
 * @brief   Handles different states/modes and commands of the e-puck.
 */

// C standard header files

#include <stdio.h>
#include <stdlib.h>

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
#include <mod_calibration.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

// List of possible commands sent from the computer

#define CMD_RESET          'R'
#define CMD_PAUSE          'P'
#define CMD_CONTINUE       'C'
#define CMD_CALIBRATE      'B'
#define CMD_GET_DATA       'G'
#define CMD_DRAW           'D'
#define CMD_INTERACTIVE    'I'
#define CMD_HOME           'H'
#define CMD_VALIDATE       'V'


// Periods

#define CMD_PERIOD         100

/*===========================================================================*/
/* Module thread pointers.                                                   */
/*===========================================================================*/

static thread_t* ptr_process_cmd;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/


/**
 * @brief               Processes command and calls relevant module functions.
 *
 * @param[in]   cmd     Command. Possible commands listed in module constants.
 */
static void process_command(uint8_t cmd)
{
	switch(cmd) {
		case CMD_RESET:
			data_free();
			draw_stop_thd();
			cal_stop_thd();
			draw_reset();
			draw_set_init_length(100);
			break;
		case CMD_PAUSE:
			draw_pause_thd();
			break;
		case CMD_CONTINUE:
			draw_resume_thd();
			cal_signal_changed_colors();
			break;
		case CMD_CALIBRATE:
			cal_create_thd();
			break;
		case CMD_GET_DATA:
			//capture_image()
			com_receive_data((BaseSequentialStream *)&SD3);
			break;
		case CMD_DRAW:
			draw_create_thd();
			break;
		case CMD_INTERACTIVE:
			break;
		case CMD_HOME:
			draw_move(512, 0);
			break;
		case CMD_VALIDATE:
			cal_set_goal_distance();
			break;
	}
}

/*===========================================================================*/
/* Module threads.                                                           */
/*===========================================================================*/

/**
 * @brief                Thread for reading command from serial (SD3).
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
	ptr_process_cmd = chThdCreateStatic(wa_process_cmd, sizeof(wa_process_cmd),
	                                    NORMALPRIO+1, thd_process_cmd, NULL);
}



