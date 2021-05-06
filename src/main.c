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


#include <camera/po8030.h>

#include "camera/dcmi_camera.h"







// Module headers

#include <main.h>
#include <mod_communication.h>
#include "modules/include/mod_img_processing.h"
#include "modules/include/mod_path.h"
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

#define PERIOD_CMD		1000


/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

//just a test
static void send_path(void) // just a test, to include with communication functions in mod_state branch
{
	uint16_t length = data_get_length();
	cartesian_coord* path = data_get_pos();
	uint8_t* color = data_get_color();

	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START\r", 6);
	chprintf((BaseSequentialStream *)&SD3, "path\n");
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&length, sizeof(uint16_t));
	for(uint16_t i = 0; i<length; ++i) {
		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&(path[i].x), sizeof(uint8_t));
	}
	for(uint16_t i = 0; i<length; ++i) {
		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&(path[i].y), sizeof(uint8_t));
	}
	for(uint16_t i = 0; i<length; ++i) {
		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&(color[i]), sizeof(uint8_t));
	}
}

/**
 * @brief				Initalizes all modules.
 */
static void init_all(void)
{
	halInit();
	chSysInit();
	mpu_init();
	usb_start();
	dcmi_start();
	po8030_start();
	com_serial_start();
	mod_img_processing_init();
}

/**
 * @brief				Processes command and calls relevant module functions.
 *
 * @param[in] 	cmd 	Command. Possible commands listed in module constants.
 */
static void process_command(uint8_t cmd)
{

//	uint8_t* color = (uint8_t*)malloc(IM_LENGTH_PX*IM_HEIGHT_PX*sizeof(uint8_t));

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
//			chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
//			img_buffer = dcmi_get_last_image_ptr();
//			chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer, 4000);
//			chThdSleepMilliseconds(400);
//			chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+4000, 4000);
//			chThdSleepMilliseconds(400);
//			chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+8000, 4000);
//			chThdSleepMilliseconds(400);
//			chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+12000, 4000);
//			chThdSleepMilliseconds(400);
//			chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+16000, 2000);
//			chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+70*100, 70*100);
//			send_image_half(img_buffer);
//			send_image();
//			send_image_half();
			send_path();
			break;
		case CMD_GET_DATA:
//			com_receive_data((BaseSequentialStream *)&SD3);
			break;
		case CMD_DRAW:
			capture_image();
			chprintf((BaseSequentialStream *)&SDU1, "draw \r \n");
//			path=path_planning();
//			path_planning();
			// Path te renvoit les coordonn�es x et y du trajet � parcourir...normalement.
			// color  change sa taille et a dor�navant la m�me taille que path. Chaque point a sa propre couleur
			// assign�e.
			chprintf((BaseSequentialStream *)&SDU1, "Image captured \r \n");
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
