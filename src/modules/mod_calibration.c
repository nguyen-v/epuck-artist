/**
 * @file    mod_calibration.c
 * @brief   Module for managing sensors
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

#include <mod_calibration.h>
#include <mod_sensors.h>
#include <mod_communication.h>
#include <mod_data.h>
#include <def_epuck_field.h>

// test
#include <motors.h>
#include "arm_math.h"
#include <mod_draw.h>

#define CALIBRATION_SPEED 		400 // steps/s
#define CALIBRATION_STEPS		200

//// e-puck
//#define SPOOL_DIAMETER     		2.1f 	 // cm
//#define SPOOL_PERIMETER			(PI*SPOOL_DIAMETER) // cm
//#define NSTEP_ONE_TURN			1000.0f
//
//// field geometry
//#define SUPPORT_DISTANCE 		48.0f // cm
//#define SPOOL_DISTANCE     		8.0f    // cm
//#define CM_TO_STEP				(NSTEP_ONE_TURN/SPOOL_PERIMETER)



#define DIST_TOF_FORWARD_LOW		0 //mm
#define DIST_TOF_FORWARD_HIGH		150 //mm
#define DIST_TOF_BACKWARD_LOW		200	//mm
#define DIST_TOF_BACKWARD_HIGH		400 //mm
#define DIST_TOF_DIFF_THR			50

#define LENGTH_DIST_SLOPE			0.0948f 	// found experimentally
#define LENGTH_DIST_INTERCEPT		-14.189f 	// found experimentally
#define CORRECTION_FACTOR 			1.0f

static bool is_calibrating = false;

/*===========================================================================*/
/* Module thread pointers.                                                   */
/*===========================================================================*/

static thread_t* ptr_calibrate;


/*===========================================================================*/
/* Module threads.                                                   		 */
/*===========================================================================*/

static THD_WORKING_AREA(wa_calibrate, 1024);
static THD_FUNCTION(thd_calibrate, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	// set arbitrary initial height
	draw_set_init_length((SUPPORT_DISTANCE-SPOOL_DISTANCE)/2);

	// draw a 100 px line
	draw_move(612, 0);
	draw_move(512, 0);
	uint16_t dist = 0;
	uint16_t prev_dist = 0;
	int16_t step_init = (left_motor_get_pos()+right_motor_get_pos())/2;
	int16_t step = step_init;

	// calculate expected length difference for perfect square
	uint16_t expected_diff_length = draw_get_length_av_next(512, 100) - draw_get_length_av_current();
	uint16_t true_diff_length = 0;

	// turn on front led
	palSetPad(GPIOD, GPIOD_LED_FRONT);

	while(!chThdShouldTerminateX()) {
		// get current length and distance
		dist = sensors_tof_kalman();
		step = (left_motor_get_pos()+right_motor_get_pos())/2;

		if(dist>DIST_TOF_FORWARD_LOW && dist < DIST_TOF_FORWARD_HIGH
				&& abs(dist-prev_dist) < DIST_TOF_DIFF_THR && step < step_init) {
			left_motor_set_speed(CALIBRATION_SPEED*(DIST_TOF_FORWARD_HIGH-dist)/DIST_TOF_FORWARD_HIGH);
			right_motor_set_speed(CALIBRATION_SPEED*(DIST_TOF_FORWARD_HIGH-dist)/DIST_TOF_FORWARD_HIGH);
		} else if (dist>DIST_TOF_BACKWARD_LOW && dist < DIST_TOF_BACKWARD_HIGH && abs(dist-prev_dist) < DIST_TOF_DIFF_THR) {
			left_motor_set_speed(-CALIBRATION_SPEED*(dist-DIST_TOF_BACKWARD_LOW)/(DIST_TOF_BACKWARD_HIGH-DIST_TOF_BACKWARD_LOW));
			right_motor_set_speed(-CALIBRATION_SPEED*(dist-DIST_TOF_BACKWARD_LOW)/(DIST_TOF_BACKWARD_HIGH-DIST_TOF_BACKWARD_LOW));
		} else {
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}

		//
		true_diff_length = step_init - (left_motor_get_pos()+right_motor_get_pos())/2;
		chprintf((BaseSequentialStream *)&SDU1, "true length %d \r \n", true_diff_length);
		chprintf((BaseSequentialStream *)&SDU1, "expected length %d \r \n", expected_diff_length);
		chThdSleepMilliseconds(100);
		prev_dist = dist;
	}
	is_calibrating = false;
	palClearPad(GPIOD, GPIOD_LED_FRONT);

	// return true length difference on exit
	chThdExit((msg_t)true_diff_length);
}

void cal_create_thd(void)
{
	if (!is_calibrating) {
		ptr_calibrate = chThdCreateStatic(wa_calibrate, sizeof(wa_calibrate), NORMALPRIO, thd_calibrate, NULL);
		is_calibrating = true;
	}
}

static uint16_t cal_stop_thd(void)
{
	chThdTerminate(ptr_calibrate);
	msg_t true_diff_length = chThdWait(ptr_calibrate);
	is_calibrating = false;
	// return true length difference (steps)
	return (uint16_t)true_diff_length;
}

void cal_set_init_length(void)
{
	uint16_t true_diff_length = cal_stop_thd();
	float corrected_length = (true_diff_length * LENGTH_DIST_SLOPE + LENGTH_DIST_INTERCEPT)*CORRECTION_FACTOR;
	chprintf((BaseSequentialStream *)&SDU1, "corrected length %f \r \n", corrected_length);
//	draw_set_init_length();
}

float get_initial_height_cm(void)
{
	// get initial height and length
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	uint16_t step_left = left_motor_get_pos();
	uint16_t step_right = right_motor_get_pos();
	uint16_t step_left_fin = 0;
	uint16_t step_right_fin = 0;
	uint16_t dist = sensors_tof_kalman();
	uint16_t dist_fin = 0;
	chprintf((BaseSequentialStream *)&SDU1, "init step left %d \r \n", step_left);
	chprintf((BaseSequentialStream *)&SDU1, "init step right %d \r \n", step_right);
	chprintf((BaseSequentialStream *)&SDU1, "init dist mm %d \r \n", dist);
	// move down
	chThdSleepSeconds(2);
//	left_motor_set_speed(-CALIBRATION_SPEED);
//	right_motor_set_speed(-CALIBRATION_SPEED);
//	chThdSleepMilliseconds(1000*CALIBRATION_STEPS/CALIBRATION_SPEED);
//	left_motor_set_speed(0);
//	right_motor_set_speed(0);
//	chThdSleepSeconds(2);
//
//	// get updated height and length
//	step_left -=  left_motor_get_pos();
//	step_right -= right_motor_get_pos();
//	dist_fin = sensors_tof_kalman();
//	printf((BaseSequentialStream *)&SDU1, "init dist mm %d \r \n", dist_fin);
//	dist -= dist_fin;
//	chprintf((BaseSequentialStream *)&SDU1, "fin step left %d \r \n", step_left);
//	chprintf((BaseSequentialStream *)&SDU1, "fin step right %d \r \n", step_right);
//
//	float dy = dist/10.; // conversion to cm
//	float dl = (step_left + step_right)/(2*CM_TO_STEP);
//	chprintf((BaseSequentialStream *)&SDU1, "fin dy cm %f \r \n", dy);
//	chprintf((BaseSequentialStream *)&SDU1, "updated length %f \r \n", dl);
//	chThdSleepSeconds(1);

	for (uint8_t i = 0; i< 5; ++i) {
		left_motor_set_speed(-CALIBRATION_SPEED);
		right_motor_set_speed(-CALIBRATION_SPEED);
		chThdSleepMilliseconds(1000*CALIBRATION_STEPS/CALIBRATION_SPEED);
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		chThdSleepMilliseconds(3000);
		dist_fin = sensors_tof_kalman();
		step_left_fin = step_left - left_motor_get_pos();
		step_right_fin = step_right - right_motor_get_pos();
//		chprintf((BaseSequentialStream *)&SDU1, "dy %f \r \n", (dist-dist_fin)/10.);
//		chprintf((BaseSequentialStream *)&SDU1, "dl %f \r \n", (step_left_fin + step_right_fin)/(2*CM_TO_STEP));
		chprintf((BaseSequentialStream *)&SDU1, "%f %f \r \n", (dist-dist_fin)/10.,(step_left_fin + step_right_fin)/(2*CM_TO_STEP));

	}

	// move back up
	left_motor_set_speed(CALIBRATION_SPEED);
	right_motor_set_speed(CALIBRATION_SPEED);
	chThdSleepMilliseconds(1000*5*CALIBRATION_STEPS/CALIBRATION_SPEED);
	left_motor_set_speed(0);
	right_motor_set_speed(0);

	// calculate inital height
//	float dx_sq = dl*dl - dy*dy;
//	float x = (SUPPORT_DISTANCE-SPOOL_DISTANCE)/2.;
//	chprintf((BaseSequentialStream *)&SDU1, "X %f \r \n", x);
//	chprintf((BaseSequentialStream *)&SDU1, "dxsq %f \r \n", dx_sq);
//	return -(dl*sqrt(-dx_sq*(4*x*x-dx_sq))+dx_sq*dy)/(2.*dx_sq);
//	return -(dl*sqrt(-dx_sq*(4*x*x-dx_sq))-dx_sq*dy)/(2.*dx_sq);
	return 1;
}

//void calibration_height(void)
//{
//
//}
