/**
 * @file    main.c
 * @brief   Main file. Handles interactions between modules.
 */

// ChibiOS headers

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "chprintf.h"
#include <usbcfg.h>
#include <motors.h>

// e-puck 2 main processor headers

#include <camera/po8030.h>
#include "camera/dcmi_camera.h"


// Module headers

#include <main.h>
#include <mod_sensors.h>
#include <mod_draw.h>
#include <mod_communication.h>
#include <mod_img_processing.h>
#include <mod_calibration.h>
#include <mod_data.h>
#include <mod_state.h>

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief             Initalizes all modules.
 */
static void init_all(void)
{
	halInit();
	chSysInit();
	mpu_init();
//	usb_start(); // uncomment if using chprintf on SDU1
	dcmi_start();
	po8030_start();
	com_serial_start();
	mod_img_processing_init();
	motors_init();
	sensors_init();
}

/*===========================================================================*/
/* Main function.                                                   		 */
/*===========================================================================*/


int main(void)
{
	init_all();
	draw_set_init_length(DEFAULT_HEIGHT_CM);
	create_thd_process_cmd();
	while(1) {
		chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
