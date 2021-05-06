/**
 * @file    mod_calibration.c
 * @brief   Module for managing sensors
 */

// C standard header files

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// ChibiOS headers

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "chprintf.h"
#include <usbcfg.h>

// e-puck2 main processor headers

#include <motors.h>

// Module headers

#include <mod_calibration.h>
#include <mod_draw.h>
#include <mod_sensors.h>
#include <mod_communication.h>
#include <mod_data.h>
#include <def_epuck_field.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

// Calibration

#define CALIBRATION_SPEED          400     // steps/s
#define CALIBRATION_STEPS          200
#define CALIBRATION_PERIOD         100     // ms
#define CALIBRATION_SQ_PX          100     // distance between calibration points
                                            // in pixels

#define KP                         10.f    // Proportional controller coefficient
#define TOF_DISTANCE_MIN           150     // mm
#define TOF_DISTANCE_MAX           300     // mm
#define TOF_PRECISION_THRESHOLD    2       // mm
#define TOF_STEADY_INTERVAL        3000    // ms

// TOF distance thresholds in mm

#define DIST_TOF_FORWARD_LOW       0
#define DIST_TOF_FORWARD_HIGH      150
#define DIST_TOF_BACKWARD_LOW      200
#define DIST_TOF_BACKWARD_MID      350
#define DIST_TOF_BACKWARD_HIGH     450
#define DIST_TOF_DIFF_THR          50

// Initial height/length calculation

#define LENGTH_DIST_SLOPE          52.7f   // found experimentally
#define LENGTH_DIST_INTERCEPT      -14.5f  // found experimentally
#define CORRECTION_FACTOR          0.92f   // to compensate model error when
                                            // setting initial length

// distance measured by TOF sensor has to be multiplied by this factor
// because the epuck is not completely parallel to the wall

#define TOF_CORRECTION_FACTOR      1.1823

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static bool is_calibrating = false;
static bool reached_goal_distance = false;
static bool reached_home = false;

/*===========================================================================*/
/* Module thread pointers.                                                   */
/*===========================================================================*/

static thread_t* ptr_calibrate;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief                        P controller on speed to reach goal distance
 * @param[in]  goal_distance     Distance to reach in mm
 * @param[in]  init_distance     Initial (starting) distance in mm
 * @return                       none
 */
static int16_t get_speed_p(int16_t goal_distance, int16_t init_distance)
{
	int16_t speed = 0;
	int16_t error = 0;
	int16_t distance = init_distance - sensors_tof_kalman();
	error = distance - goal_distance;

	if (error < -TOF_DISTANCE_MAX)
		return 0;

	if (abs(error) < TOF_PRECISION_THRESHOLD) {
		reached_goal_distance = true;
		return 0;
	}

	speed = KP * error;

	// define lower and upped bounds for the speed returned
	if(speed > CALIBRATION_SPEED)
		speed = CALIBRATION_SPEED;
	if(speed < -CALIBRATION_SPEED)
		speed = -CALIBRATION_SPEED;

	return (int16_t)speed;
}


/**
 * @brief                        P controller on motor step count to
 *                               reach initial position
 * @return                       none
 */
static int16_t move_home(void)
{
	float speed = 0;
	int16_t error = -(left_motor_get_pos()+right_motor_get_pos())/2;
	if (abs(error) < TOF_PRECISION_THRESHOLD*MM_TO_STEP) {
		reached_home = true;
	}
	speed = KP/MM_TO_STEP * error;

	// define lower and upped bounds for the speed returned
	if(speed > CALIBRATION_SPEED)
		speed = CALIBRATION_SPEED;
	if(speed < -CALIBRATION_SPEED)
		speed = -CALIBRATION_SPEED;
	return (int16_t)speed;
}

/**
 * @brief                            Sets the initial distance between e-puck and
 *                                   the line between the 2 thread attach points
 * @param[in]  true_diff_length      True distance in step measured during calculation
 * @param[in]  expected_diff_length  Expected length in step during calibration
 * @return                           none
 * @note                             LENGTH_DIST_INTERCEPT and CORRECTION_FACTOR
 *                                   were found experimentally by plotting the
 *                                   diff_length_ratio against the true initial lenth.
 */
static void set_init_length(uint16_t true_diff_length, uint16_t expected_diff_length)
{
	float diff_length_ratio = (float)true_diff_length/expected_diff_length;
	float corrected_length = (diff_length_ratio * LENGTH_DIST_SLOPE +
	                          LENGTH_DIST_INTERCEPT)*CORRECTION_FACTOR;
	draw_set_init_length(corrected_length*CORRECTION_FACTOR);
}

/*===========================================================================*/
/* Module threads.                                                   		 */
/*===========================================================================*/

/**
 * @brief     Calibration of initial distance between e-puck
 *            and the line between the 2 thread attach points
 * @detail    It is crucial to correctly calibrate the initial distance in order
 *            to avoid distortions of the  final picture. An estimation that is
 *            too high or low could result in the drawn picture being squashed
 *            or stretched.
 *            First, the robot draws 2 calibration points. The user can then
 *            physically measure the distance between the 2 calibration points
 *            and give the distance measured to the robot. The robot then goes
 *            down by this distance and compares its expected travel distance
 *            calculated from an arbitrary initial distance with the true travel
 *            distance. True initial position can then be calculated.
 *
 */

static THD_WORKING_AREA(wa_calibrate, 1024);
static THD_FUNCTION(thd_calibrate, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	// set arbitrary initial height
	// here we suppose that the e-puck is at a 45 degree angle with the
	// supporting threads
	draw_set_init_length((SUPPORT_DISTANCE-SPOOL_DISTANCE)/2);

	// reset left and right motor
	draw_reset();

	// draw first calibration point
	com_request_color(black);
	com_request_color(white);
	draw_move(X_DEFAULT+CALIBRATION_SQ_PX, Y_DEFAULT);

	// draw second calibration point
	com_request_color(black);
	com_request_color(white);
	draw_move(X_DEFAULT, Y_DEFAULT);

	// calculate expected vertical distance in steps that the robot needs to
	// travel to equal the horizontal distance between the 2 calibration points
	uint16_t expected_diff_length = draw_get_length_av_next(X_DEFAULT, Y_DEFAULT
	                                + CALIBRATION_SQ_PX) - draw_get_length_av_current();
	uint16_t true_diff_length = 0;
	int16_t speed = 0;

	// turn on front led
	palSetPad(GPIOD, GPIOD_LED_FRONT);

	// wait and get goal distance value from computer
	thread_t *tp = chMsgWait();
	msg_t goal_distance = chMsgGet(tp);
	chMsgRelease(tp, MSG_OK);

	uint16_t init_distance = 0;

	// turn off front led
	palClearPad(GPIOD, GPIOD_LED_FRONT);

	if (!chThdShouldTerminateX()) {
		// wait until distance is steady for long enough
		init_distance = sensors_tof_wait(TOF_DISTANCE_MIN, TOF_DISTANCE_MAX,
		                                 TOF_PRECISION_THRESHOLD, TOF_STEADY_INTERVAL);
		reached_goal_distance = false;
		reached_home = false;
	}
	while (!chThdShouldTerminateX()) {

		chThdSleepMilliseconds(CALIBRATION_PERIOD);

		if (!reached_goal_distance) {
			// move to goal distance and update true travel distance
			speed = get_speed_p(TOF_CORRECTION_FACTOR*goal_distance, init_distance);
			true_diff_length = -(left_motor_get_pos()+right_motor_get_pos())/2;
			left_motor_set_speed(speed);
			right_motor_set_speed(speed);

		} else if (!reached_home) {
			// move back to starting point and terminate thread
			speed = move_home();
			left_motor_set_speed(speed);
			right_motor_set_speed(speed);

		} else {
			// calculate and set initial distance
			draw_reset();
			set_init_length(true_diff_length, expected_diff_length);
			chThdTerminate(ptr_calibrate);
		}

	}

	is_calibrating = false;;

	// return true length difference on exit
	chThdExit((msg_t)true_diff_length);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void cal_set_goal_distance(void)
{
	if (is_calibrating) {
		uint8_t goal_distance = com_receive_length((BaseSequentialStream *)&SD3);
		(void)chMsgSend(ptr_calibrate, (msg_t)goal_distance);
	}
}

void cal_create_thd(void)
{
	if (!is_calibrating) {
		ptr_calibrate = chThdCreateStatic(wa_calibrate, sizeof(wa_calibrate), NORMALPRIO, thd_calibrate, NULL);
		is_calibrating = true;
	}
}

uint16_t cal_stop_thd(void)
{
	if (is_calibrating) {

		// send message to unblock chMsgWait()
		(void)chMsgSend(ptr_calibrate, 0);

		chThdTerminate(ptr_calibrate);
		uint16_t true_diff_length = chThdWait(ptr_calibrate);
		is_calibrating = false;

		// return true length difference (steps)
		return true_diff_length;
	}
	return 0;
}

bool cal_get_state(void)
{
	return is_calibrating;
}

//void cal_set_init_length(void)
//{
//	uint16_t true_diff_length = cal_stop_thd();
//	if (true_diff_length != 0) {
//		draw_move(X_DEFAULT, Y_DEFAULT);
//		float diff_length_ratio = (float)true_diff_length/expected_diff_length;
//		float corrected_length = (diff_length_ratio * LENGTH_DIST_SLOPE + LENGTH_DIST_INTERCEPT)*CORRECTION_FACTOR;
//		draw_set_init_length(corrected_length*CORRECTION_FACTOR);
//		chprintf((BaseSequentialStream *)&SDU1, "corrected length %f \r \n", corrected_length);
//	}
////	draw_set_init_length();
//}


//float get_initial_height_cm(void)
//{
//	// get initial height and length
//	left_motor_set_pos(0);
//	right_motor_set_pos(0);
//	uint16_t step_left = left_motor_get_pos();
//	uint16_t step_right = right_motor_get_pos();
//	uint16_t step_left_fin = 0;
//	uint16_t step_right_fin = 0;
//	uint16_t dist = sensors_tof_kalman();
//	uint16_t dist_fin = 0;
//	chprintf((BaseSequentialStream *)&SDU1, "init step left %d \r \n", step_left);
//	chprintf((BaseSequentialStream *)&SDU1, "init step right %d \r \n", step_right);
//	chprintf((BaseSequentialStream *)&SDU1, "init dist mm %d \r \n", dist);
//	// move down
//	chThdSleepSeconds(2);
////	left_motor_set_speed(-CALIBRATION_SPEED);
////	right_motor_set_speed(-CALIBRATION_SPEED);
////	chThdSleepMilliseconds(1000*CALIBRATION_STEPS/CALIBRATION_SPEED);
////	left_motor_set_speed(0);
////	right_motor_set_speed(0);
////	chThdSleepSeconds(2);
////
////	// get updated height and length
////	step_left -=  left_motor_get_pos();
////	step_right -= right_motor_get_pos();
////	dist_fin = sensors_tof_kalman();
////	printf((BaseSequentialStream *)&SDU1, "init dist mm %d \r \n", dist_fin);
////	dist -= dist_fin;
////	chprintf((BaseSequentialStream *)&SDU1, "fin step left %d \r \n", step_left);
////	chprintf((BaseSequentialStream *)&SDU1, "fin step right %d \r \n", step_right);
////
////	float dy = dist/10.; // conversion to cm
////	float dl = (step_left + step_right)/(2*CM_TO_STEP);
////	chprintf((BaseSequentialStream *)&SDU1, "fin dy cm %f \r \n", dy);
////	chprintf((BaseSequentialStream *)&SDU1, "updated length %f \r \n", dl);
////	chThdSleepSeconds(1);
//
//	for (uint8_t i = 0; i< 5; ++i) {
//		left_motor_set_speed(-CALIBRATION_SPEED);
//		right_motor_set_speed(-CALIBRATION_SPEED);
//		chThdSleepMilliseconds(1000*CALIBRATION_STEPS/CALIBRATION_SPEED);
//		left_motor_set_speed(0);
//		right_motor_set_speed(0);
//		chThdSleepMilliseconds(3000);
//		dist_fin = sensors_tof_kalman();
//		step_left_fin = step_left - left_motor_get_pos();
//		step_right_fin = step_right - right_motor_get_pos();
////		chprintf((BaseSequentialStream *)&SDU1, "dy %f \r \n", (dist-dist_fin)/10.);
////		chprintf((BaseSequentialStream *)&SDU1, "dl %f \r \n", (step_left_fin + step_right_fin)/(2*CM_TO_STEP));
//		chprintf((BaseSequentialStream *)&SDU1, "%f %f \r \n", (dist-dist_fin)/10.,(step_left_fin + step_right_fin)/(2*CM_TO_STEP));
//
//	}
//
//	// move back up
//	left_motor_set_speed(CALIBRATION_SPEED);
//	right_motor_set_speed(CALIBRATION_SPEED);
//	chThdSleepMilliseconds(1000*5*CALIBRATION_STEPS/CALIBRATION_SPEED);
//	left_motor_set_speed(0);
//	right_motor_set_speed(0);
//
//	// calculate inital height
////	float dx_sq = dl*dl - dy*dy;
////	float x = (SUPPORT_DISTANCE-SPOOL_DISTANCE)/2.;
////	chprintf((BaseSequentialStream *)&SDU1, "X %f \r \n", x);
////	chprintf((BaseSequentialStream *)&SDU1, "dxsq %f \r \n", dx_sq);
////	return -(dl*sqrt(-dx_sq*(4*x*x-dx_sq))+dx_sq*dy)/(2.*dx_sq);
////	return -(dl*sqrt(-dx_sq*(4*x*x-dx_sq))-dx_sq*dy)/(2.*dx_sq);
//	return 1;
//}

//void calibration_height(void)
//{
//
//}


// get current length and distance
//		dist = sensors_tof_kalman();
//		step = (left_motor_get_pos()+right_motor_get_pos())/2;
//
//		if(dist>DIST_TOF_FORWARD_LOW && dist < DIST_TOF_FORWARD_HIGH
//				&& abs(dist-prev_dist) < DIST_TOF_DIFF_THR && step < step_init) {
//			left_motor_set_speed(CALIBRATION_SPEED*(DIST_TOF_FORWARD_HIGH-dist)/DIST_TOF_FORWARD_HIGH);
//			right_motor_set_speed(CALIBRATION_SPEED*(DIST_TOF_FORWARD_HIGH-dist)/DIST_TOF_FORWARD_HIGH);
//		} else if (dist>DIST_TOF_BACKWARD_LOW && dist < DIST_TOF_BACKWARD_MID && abs(dist-prev_dist) < DIST_TOF_DIFF_THR) {
//			left_motor_set_speed(-CALIBRATION_SPEED*(dist-DIST_TOF_BACKWARD_LOW)/(DIST_TOF_BACKWARD_MID-DIST_TOF_BACKWARD_LOW));
//			right_motor_set_speed(-CALIBRATION_SPEED*(dist-DIST_TOF_BACKWARD_LOW)/(DIST_TOF_BACKWARD_MID-DIST_TOF_BACKWARD_LOW));
//		} else if (dist>DIST_TOF_BACKWARD_MID && dist < DIST_TOF_BACKWARD_HIGH && abs(dist-prev_dist) < DIST_TOF_DIFF_THR) {
//			left_motor_set_speed(-CALIBRATION_SPEED*(DIST_TOF_BACKWARD_HIGH-dist)/(DIST_TOF_BACKWARD_HIGH-DIST_TOF_BACKWARD_MID));
//			right_motor_set_speed(-CALIBRATION_SPEED*(DIST_TOF_BACKWARD_HIGH-dist)/(DIST_TOF_BACKWARD_HIGH-DIST_TOF_BACKWARD_MID));
//		} else {
//			left_motor_set_speed(0);
//			right_motor_set_speed(0);
//		}
//
//		//
//		true_diff_length = step_init - (left_motor_get_pos()+right_motor_get_pos())/2;
//		chprintf((BaseSequentialStream *)&SDU1, "true length %d \r \n", true_diff_length);
//		chprintf((BaseSequentialStream *)&SDU1, "expected length %d \r \n", expected_diff_length);
