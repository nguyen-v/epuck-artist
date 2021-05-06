/**
 * @file    main.c
 * @brief   Main file. Handles interactions between modules.
 */

// C standard header files
//#include <stdio.h>
//#include <stdlib.h>

// ChibiOS headers

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "chprintf.h"
#include <usbcfg.h>
#include <motors.h>

// Module headers

#include <main.h>
#include <mod_sensors.h>
#include <mod_draw.h>
#include <mod_communication.h>
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
//	usb_start();
	com_serial_start();
	motors_init();
	sensors_init();
}

/*===========================================================================*/
/* Main function.                                                   		 */
/*===========================================================================*/


int main(void)
{
	init_all();
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
