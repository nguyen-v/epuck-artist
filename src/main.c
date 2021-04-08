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
#include <mod_data.h>

// test
#include <motors.h>
#include "arm_math.h"
#include <mod_draw.h>
#include "sensors/VL53L0X/VL53L0X.h"

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
	motors_init();
//	VL53L0X_start();
}

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
			palClearPad(GPIOD, GPIOD_LED1);
			chThdSleepMilliseconds(1000);
			palSetPad(GPIOD, GPIOD_LED1);
			chprintf((BaseSequentialStream *)&SDU1, "RESET \r \n");
			break;
		case CMD_PAUSE:
			pos = data_get_pos();
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
//				draw_move(pos[i].x, pos[i].y);
//				chThdSleepMilliseconds(500);
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


static void timer11_start(void){
    //General Purpose Timer configuration
    //timer 11 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt11cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD11, &gpt11cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD11, 0xFFFF);
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
//	timer11_start();
	//	chThdCreateStatic(wa_process_cmd, sizeof(wa_process_cmd), NORMALPRIO, thd_process_cmd, NULL);
//	volatile uint16_t time = 0;
//
//	draw_set_init_length();
//	draw_reset();
	chThdSleepMilliseconds(2000);
//	draw_move(0, 0);

	while(1) {
		process_command(com_receive_command((BaseSequentialStream *)&SD3));
		chThdSleepMilliseconds(CMD_PERIOD);
//		process_command(com_receive_command((BaseSequentialStream *)&SD3));
//			chprintf((BaseSequentialStream *)&SDU1, "right motor step %d \r \n", right_motor_get_pos());
//		int16_t a = 432;
//	    int16_t b = 6546;
//	    uint32_t sum = 0;
//	    chSysLock();
//	    GPTD11.tim->CNT = 0;
//	    for(uint8_t i=0;i<100;++i) {
//			sum = (a*a + b*b)<<1;
//			arm_sqrt_q31((q31_t)sum, (q31_t*)&sum);
//			sum >>= 16;
//	    }
//	    time = GPTD11.tim->CNT;
//	    chSysUnlock();
//
//	    chprintf((BaseSequentialStream *)&SDU1, "time 1 = %dus\n",time);
//	    chprintf((BaseSequentialStream *)&SDU1, "sqrt 1 = %d \r \n",sum);
//
//	    chSysLock();
//	    GPTD11.tim->CNT = 0;
//	    for(uint8_t i=0;i<100;++i) {
//			sum = sqrt(a*a + b*b);
//	    }
//	    time = GPTD11.tim->CNT;
//	    chSysUnlock();
//
//	    chprintf((BaseSequentialStream *)&SDU1, "time 2 = %dus\n",time);
//	    chprintf((BaseSequentialStream *)&SDU1, "sqrt 2 = %d \r \n",sum);
//
//	    int16_t u = -435;
//	    int16_t r = 0;
//	    chSysLock();
//	    GPTD11.tim->CNT = 0;
//	    for(uint8_t i=0;i<100;++i) {
//			r = abs(u+i);
//	    }
//	    time = GPTD11.tim->CNT;
//	    chSysUnlock();
//	    chprintf((BaseSequentialStream *)&SDU1, "time 1 = %dus\n",time);
//	    chprintf((BaseSequentialStream *)&SDU1, "abs 1 = %d \r \n",r);
//
//	    chSysLock();
//		GPTD11.tim->CNT = 0;
//		for(uint8_t i=0;i<100;++i) {
//			arm_abs_q15(&u, &r, 1);
//		}
//		time = GPTD11.tim->CNT;
//		chSysUnlock();
//		chprintf((BaseSequentialStream *)&SDU1, "time 2 = %dus\n",time);
//		chprintf((BaseSequentialStream *)&SDU1, "abs 2 = %d \r \n",r);





    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
