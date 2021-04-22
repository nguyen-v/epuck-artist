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
#include <mod_state.h>

// test
#include <mod_sensors.h>
#include <motors.h>
#include "arm_math.h"
#include <mod_draw.h>
#include "msgbus/messagebus.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/proximity.h"

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
	sensors_init();
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
	create_thd_process_cmd();
//	create_thd_draw();
//	draw_move(0, 0);

	uint16_t dist = 0;
	uint16_t dist_kalman = 0;
	uint16_t dist_kalman2 = 0;
	uint16_t dist_kalman3 = 0;

	int prox1 = 0;
	int prox3 = 0;
	int prox5 = 0;
	int prox7 = 0;


	while(1) {
//		dist = sensors_tof_kalman1d();
//		chprintf((BaseSequentialStream *)&SDU1, "$%d;", dist);
//		dist_av = dsp_ema_i32(dist, dist_av, DSP_EMA_I32_ALPHA(0.1));
//		dist_av2 = dsp_ema_i32(dist, dist_av, DSP_EMA_I32_ALPHA(0.5));
//		dist_kalman = kalman(dist);
//		dist_kalman2 = kalman2(dist);
//		dist_kalman3 = kalman3(dist);

//		prox1 = get_prox(1);
//		prox3 = get_prox(3);
//		prox5 = get_prox(5);
//		prox7 = get_prox(7);
//		chprintf((BaseSequentialStream *)&SDU1, "$%d %d %d %d;", prox1, prox3, prox5, prox7);


//		chprintf((BaseSequentialStream *)&SDU1, "$%d %d %d %d;", dist, dist_kalman, dist_kalman2, dist_kalman3);
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
