/**
 * @file    mod_draw.c
 * @brief   Module for moving the robot to XY coordinates.
 * @note
 */

// C standard header files

#include <stdint.h>
#include <math.h>
#include <stdlib.h>

// ChibiOS headers

#include "hal.h"
#include "ch.h"
#include <motors.h>
#include <usbcfg.h>		// usb debug messages
#include "chprintf.h" 	// usb debug messages
//#include "arm_math.h"	// slower than math.h functions...

// Module headers

#include <mod_draw.h>
#include <mod_data.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define X_RESOLUTION			1024
#define PI						3.14159265358979f
// e-puck
#define SPOOL_DISTANCE     		8.0f    // cm
#define SPOOL_DIAMETER     		2.1f 	 // cm
#define SPOOL_PERIMETER			(PI*SPOOL_DIAMETER) // cm
#define NSTEP_ONE_TURN			1000.0f

// field geometry
#define SUPPORT_DISTANCE 		87.5f // cm
#define MARGIN					(SPOOL_DISTANCE*2) // cm

#define CM_TO_STEP				(NSTEP_ONE_TURN/SPOOL_PERIMETER)

// in steps
#define SPOOL_DISTANCE_ST  		((uint16_t)(SPOOL_DISTANCE*CM_TO_STEP))
#define SUPPORT_DISTANCE_ST  	((uint16_t)(SUPPORT_DISTANCE*CM_TO_STEP))
#define MARGIN_ST 				((uint16_t)(MARGIN*CM_TO_STEP))

#define CART_TO_ST				((float)(SUPPORT_DISTANCE_ST-2*MARGIN_ST)/X_RESOLUTION)

#define MIN_SPEED				80 // step/s
#define MAX_SPEED 				150 // steps/s

#define STEP_THRESHOLD			5 // defines how close we should get to
								  // goal length in steps

#define TIME_SLEEP_MIN			20 // ms motors dont have time to react if too low

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/


//static float pos_y_initial = 0;	// cm
static uint16_t y0_st = 100.0*CM_TO_STEP;
static uint16_t x0_st = (SUPPORT_DISTANCE_ST-SPOOL_DISTANCE_ST)/2.;
static uint16_t len0_st = 0; //steps

//static cartesian_coord current_pos = {X_RESOLUTION/2, 0};

/*===========================================================================*/
/* Module mutexes, semaphores.                                               */
/*===========================================================================*/
//MUTEX_DECL(serial_mtx);

/*===========================================================================*/
/* Module local functions.                                                	 */
/*===========================================================================*/

//void right_motor_set_length(uint16_t step, int16_t speed)
//{
//	int16_t speed_abs = 0;
//	arm_abs_q15(&speed, &speed_abs, 1);
//
//	right_motor_set_speed(speed);
//	chThdSleepMicroseconds(step*1000000/speed_abs); // maybe try ms
//	right_motor_set_speed(0);
//}
//
//void left_motor_set_length(uint16_t step, int16_t speed)
//{
//	int16_t speed_abs = 0;
//	arm_abs_q15(&speed, &speed_abs, 1);
//
//	left_motor_set_speed(speed);
//	chThdSleepMicroseconds(step*1000000/speed_abs); // maybe try ms
//	left_motor_set_speed(0);
//}

static void motor_set_step(int32_t step_left, int32_t step_right, uint16_t speed)
{
	// calculate absolute values for computation of sign and sleep time.
	uint16_t step_left_abs = abs(step_left);
	uint16_t step_right_abs = abs(step_right);
	uint32_t time = 0;
	float speed_left = 0;
	float speed_right = 0;
	chprintf((BaseSequentialStream *)&SDU1, "step_left = %d \r \n", step_left);
	chprintf((BaseSequentialStream *)&SDU1, "step_right= %d \r \n", step_right);

	//
//	if (step_left_abs < STEP_THRESHOLD && step_right_abs > STEP_THRESHOLD) {
//		speed_left = 0;
//		speed_right = speed * (step_right/step_right_abs);
//		time = 1000*step_right_abs/speed;
//	} else if (step_left_abs > STEP_THRESHOLD && step_right_abs < STEP_THRESHOLD) {
//		speed_right = 0;
//		speed_left = speed * (step_left/step_left_abs);
//		time = 1000*step_left_abs/speed;
//	if (step_left_abs > step_right_abs) {
//		time = 1000*step_right_abs/speed;
//		speed_left = speed*step_left/step_right_abs;
//		speed_right = speed * (step_right/step_right_abs);
//		chprintf((BaseSequentialStream *)&SDU1, "left speed = %d \r \n", (int16_t)speed_left);
//		chprintf((BaseSequentialStream *)&SDU1, "right speed = %d \r \n", (int16_t)speed_right);
//		if(abs(speed_left) > MOTOR_SPEED_LIMIT) {
//			speed_left = speed * (step_left/step_left_abs);
//			speed_right = 0;
//			time = 1000*step_left_abs/speed;
//		}
//	} else {
//		time = 1000*step_left_abs/speed;
//		speed_left = speed * (step_left/step_left_abs);
//		speed_right = speed*step_right/step_left_abs;
//		chprintf((BaseSequentialStream *)&SDU1, "left speed = %d \r \n", (int16_t)speed_left);
//		chprintf((BaseSequentialStream *)&SDU1, "right speed = %d \r \n", (int16_t)speed_right);
//		if(abs(speed_right) > MOTOR_SPEED_LIMIT) {
//			speed_left = 0;
//			speed_right = speed * (step_right/step_right_abs);
//			time = 1000*step_right_abs/speed;
//		}
//	}
//
//	chprintf((BaseSequentialStream *)&SDU1, "left speed FINAL= %d \r \n", (int16_t)speed_left);
//	chprintf((BaseSequentialStream *)&SDU1, "right speed FINAL= %d \r \n", (int16_t)speed_right);
//	chprintf((BaseSequentialStream *)&SDU1, "time = %d \r \n", time);
//	right_motor_set_speed(-speed_left);
//	left_motor_set_speed(-speed_right);
//	if(time < TIME_SLEEP_MIN)
//		time = TIME_SLEEP_MIN;
	if (step_left_abs > step_right_abs) {
		time = 1000*step_left_abs/speed;
		speed_left = speed*step_left/step_left_abs;
		speed_right = speed * ((float)step_right/step_left_abs);
		chprintf((BaseSequentialStream *)&SDU1, "left speed = %d \r \n", (int16_t)speed_left);
		chprintf((BaseSequentialStream *)&SDU1, "right speed = %d \r \n", (int16_t)speed_right);
	} else {
		time = 1000*step_right_abs/speed;
		speed_left = speed * ((float)step_left/step_right_abs);
		speed_right = speed*step_right/step_right_abs;
		chprintf((BaseSequentialStream *)&SDU1, "left speed = %d \r \n", (int16_t)speed_left);
		chprintf((BaseSequentialStream *)&SDU1, "right speed = %d \r \n", (int16_t)speed_right);
	}

	chprintf((BaseSequentialStream *)&SDU1, "time = %d \r \n", time);
	right_motor_set_speed(-speed_left);
	left_motor_set_speed(-speed_right);
	chThdSleepMilliseconds(time);
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}



//int32_t xy_to_step_left(uint16_t x, uint16_t y)
//{
//
//}
//
//int32_t xy_to_step_right(uint16_t x, uint16_t y)
//{
//
//}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void draw_set_init_length(float y_length)
{
	y0_st = CM_TO_STEP*y_length;
	len0_st = sqrt(x0_st*x0_st + y0_st*y0_st);
	chprintf((BaseSequentialStream *)&SDU1, "x0_st = %d \r \n",x0_st);
	chprintf((BaseSequentialStream *)&SDU1, "len0_st = %d \r \n",len0_st);
}

void draw_reset(void)
{
	right_motor_set_pos(0);
	left_motor_set_pos(0);
}

void draw_move(uint16_t x, uint16_t y)
{
	// get current wire lengths in steps
	// Increase in step -> Decrease in wire length (because of e-puck orientation)
	// Left motor in charge of the right wire and right motor in charge of the left wire
	uint16_t len_l_current = len0_st - right_motor_get_pos();
	uint16_t len_r_current = len0_st - left_motor_get_pos();

//	chprintf((BaseSequentialStream *)&SDU1, "len_l_current = %d \r \n",len_l_current);
//	chprintf((BaseSequentialStream *)&SDU1, "len_r_current = %d \r \n",len_r_current);
//	chprintf((BaseSequentialStream *)&SDU1, "spool perim cm = %f \r \n",SPOOL_PERIMETER);
//	chprintf((BaseSequentialStream *)&SDU1, "CMTOSTEP = %f \r \n",CM_TO_STEP);
//	chprintf((BaseSequentialStream *)&SDU1, "CARTTOST = %f \r \n",CART_TO_ST);
//	chprintf((BaseSequentialStream *)&SDU1, "spool dist = %d \r \n",SPOOL_DISTANCE_ST);


//	chprintf((BaseSequentialStream *)&SDU1, "l motor = %d \r \n",left_motor_get_pos());
//	chprintf((BaseSequentialStream *)&SDU1, "r motor = %d \r \n",right_motor_get_pos());

	// calculate coordinates relative to top-left attach point
	uint16_t x_st = x*CART_TO_ST;
	uint16_t y_st = y*CART_TO_ST + y0_st;
	uint16_t x_l_st = x_st + MARGIN_ST - SPOOL_DISTANCE_ST/2;
	uint16_t x_r_st = SUPPORT_DISTANCE_ST - SPOOL_DISTANCE_ST/2 - MARGIN_ST - x_st;
//	chprintf((BaseSequentialStream *)&SDU1, "x_st = %d \r \n", x_st);
//	chprintf((BaseSequentialStream *)&SDU1, "y_st = %d \r \n", y_st);
//	chprintf((BaseSequentialStream *)&SDU1, "x_r_st = %d \r \n", x_r_st);
//	chprintf((BaseSequentialStream *)&SDU1, "x_l_st = %d \r \n", x_l_st);

	// calculate next length
	uint16_t len_l = sqrt(x_l_st*x_l_st + y_st*y_st);
	uint16_t len_r = sqrt(x_r_st*x_r_st + y_st*y_st);

	// move until next length
	while (abs(len_l-len_l_current)>STEP_THRESHOLD || abs(len_r-len_r_current)>STEP_THRESHOLD) {
		motor_set_step(len_l-len_l_current, len_r-len_r_current, MAX_SPEED);
		len_l_current = len0_st - right_motor_get_pos();
		len_r_current = len0_st - left_motor_get_pos();

//		chprintf((BaseSequentialStream *)&SDU1, "len_l = %d \r \n", len_l);
//		chprintf((BaseSequentialStream *)&SDU1, "len_r = %d \r \n", len_r);
//		chprintf((BaseSequentialStream *)&SDU1, "len_l_current = %d \r \n",len_l_current);
//		chprintf((BaseSequentialStream *)&SDU1, "len_r_current = %d \r \n",len_r_current);
//		chprintf((BaseSequentialStream *)&SDU1, "l motor = %d \r \n",left_motor_get_pos());
//		chprintf((BaseSequentialStream *)&SDU1, "r motor = %d \r \n",right_motor_get_pos());
	}

}
