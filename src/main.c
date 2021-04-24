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

// TO BE DELETED

static uint8_t *img_buffer;
static cartesian_coord *path;

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
	dcmi_start();
	po8030_start();
	com_serial_start();
}

/**
 * @brief				Processes command and calls relevant module functions.
 *
 * @param[in] 	cmd 	Command. Possible commands listed in module constants.
 */
static void process_command(uint8_t cmd)
{

	uint8_t* color = (uint8_t*)malloc(IM_LENGTH_PX*IM_HEIGHT_PX*sizeof(uint8_t));

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
//			send_image()
			break;
		case CMD_GET_DATA:
//			com_receive_data((BaseSequentialStream *)&SD3);
			break;
		case CMD_DRAW:
			capture_image(img_buffer,color);
			path=path_planning(img_buffer,color);
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
