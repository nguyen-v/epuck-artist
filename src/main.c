/**
 * @file    main.c
 * @brief   Main file. Handles interactions between modules.
 */

// C standard header files


// ChibiOS headers

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "chprintf.h"
#include <usbcfg.h>
#include <motors.h>


#include <camera/po8030.h>

#include "camera/dcmi_camera.h"







// Module headers

#include <main.h>
#include <mod_sensors.h>
#include <mod_draw.h>
#include <mod_communication.h>
#include <mod_calibration.h>
#include <mod_data.h>
#include <mod_state.h>

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
 * @brief             Initalizes all modules.
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
