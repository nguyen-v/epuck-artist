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
#include "sensors/proximity.h"

// Module headers

#include <mod_calibration.h>
#include <mod_draw.h>
#include <mod_sensors.h>
#include <mod_communication.h>
#include <mod_data.h>
#include <def_epuck_field.h>

// TEST
#include "sensors/VL53L0X/VL53L0X.h"

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

// Distance thresholds in mm for set home thread

#define DIST_TOF_FORWARD_LOW       100
#define DIST_TOF_FORWARD_HIGH      250
#define DIST_TOF_BACKWARD_LOW      200
#define DIST_TOF_BACKWARD_MID      350
#define DIST_TOF_BACKWARD_HIGH     450
#define DIST_TOF_DIFF_THR          50

#define IR1                        0
#define IR2                        1
#define IR5                        4
#define IR7                        6
#define IR8                        7
#define IR_THRESHOLD               100

#define SET_HOME_PERIOD            100     // ms

// Initial height/length calculation

#define LENGTH_DIST_SLOPE          52.7f   // found by linear regression
#define LENGTH_DIST_INTERCEPT      -14.5f  // found by linear regression
#define CORRECTION_FACTOR          0.92f   // to compensate model error when
                                           // setting initial length

// distance measured by TOF sensor has to be multiplied by this factor because
// the epuck is not completely parallel to the wall (found by linear regression)

#define TOF_CORRECTION_FACTOR      1.1823

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static bool is_calibrating = false;
static bool is_setting_home = false;
static bool is_waiting = false;
static bool is_waiting_color = false;

/*===========================================================================*/
/* Semaphores.                                                               */
/*===========================================================================*/

static BSEMAPHORE_DECL(sem_changed_color, TRUE);

/*===========================================================================*/
/* Module thread pointers.                                                   */
/*===========================================================================*/

static thread_t* ptr_calibrate;

static thread_t* ptr_set_home;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief                        P controller on speed to reach goal distance
 * @param[in]  goal_distance     Distance to reach in mm
 * @param[in]  init_distance     Initial (starting) distance in mm
 * @param[in]  reached_goal_dist Pointer to variable indicating if e-puck
 *                               has reached its goal distance
 * @return                       none
 */
static int16_t get_speed_p(int16_t goal_distance, int16_t init_distance,
                            bool* reached_goal_dist)
{
	int16_t speed = 0;
	int16_t error = 0;
	int16_t distance = init_distance - sensors_tof_kalman();

	error = distance - goal_distance;

	if (error < -TOF_DISTANCE_MAX)
		return 0;

	if (abs(error) < TOF_PRECISION_THRESHOLD) {
		*reached_goal_dist = true;
		return 0;
	}

	speed = KP * error;

	// define lower and upped bounds for the speed returned
	if (speed > CALIBRATION_SPEED)
		speed = CALIBRATION_SPEED;
	if (speed < -CALIBRATION_SPEED)
		speed = -CALIBRATION_SPEED;

	return (int16_t)speed;
}


/**
 * @brief                        P controller on motor step count to
 *                               reach initial position
 * @param[in]  reached_home      Pointer to variable indicating if e-puck
 *                               has returned to its original position
 * @return                       none
 */
static int16_t move_home(bool* reached_home)
{
	float speed = 0;
	int16_t error = -( left_motor_get_pos() + right_motor_get_pos() )/2;
	if (abs(error) < TOF_PRECISION_THRESHOLD*MM_TO_STEP) {
		*reached_home = true;
	}
	speed = KP/MM_TO_STEP * error;

	// define lower and upped bounds for the speed returned
	if (speed > CALIBRATION_SPEED)
		speed = CALIBRATION_SPEED;
	if (speed < -CALIBRATION_SPEED)
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

	is_waiting_color = true;
	com_request_color(black);
	chBSemWait(&sem_changed_color);
	is_waiting_color = false;
	if (chThdShouldTerminateX()) {
		chThdExit(0);
	}

	is_waiting_color = true;
	com_request_color(white);
	chBSemWait(&sem_changed_color);
	is_waiting_color = false;
	if (chThdShouldTerminateX()) {
		chThdExit(0);
	}

	draw_move(X_DEFAULT+CALIBRATION_SQ_PX, Y_DEFAULT);
	if (chThdShouldTerminateX()) {
		chThdExit(0);
	}

	// draw second calibration point
	is_waiting_color = true;
	com_request_color(black);
	chBSemWait(&sem_changed_color);
	is_waiting_color = false;
	if (chThdShouldTerminateX()) {
		chThdExit(0);
	}
	is_waiting_color = true;
	com_request_color(white);
	chBSemWait(&sem_changed_color);
	is_waiting_color = false;
	if (chThdShouldTerminateX()) {
		chThdExit(0);
	}
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
	is_waiting = true;
	thread_t *tp = chMsgWait();
	msg_t goal_distance = chMsgGet(tp);
	chMsgRelease(tp, MSG_OK);
	is_waiting = false;
	uint16_t init_distance = 0;

	// turn off front led
	palClearPad(GPIOD, GPIOD_LED_FRONT);

	bool reached_goal_dist = false;
	bool reached_home = false;

	if (!chThdShouldTerminateX()) {
		// wait until distance is steady for long enough
		init_distance = sensors_tof_wait(TOF_DISTANCE_MIN, TOF_DISTANCE_MAX,
		                                 TOF_PRECISION_THRESHOLD, TOF_STEADY_INTERVAL);
	}

	while (!chThdShouldTerminateX()) {

		if (!reached_goal_dist) {
			// move to goal distance and update true travel distance
			speed = get_speed_p(TOF_CORRECTION_FACTOR*goal_distance, init_distance,
			                    &reached_goal_dist);
			true_diff_length = -(left_motor_get_pos()+right_motor_get_pos())/2;
			left_motor_set_speed(speed);
			right_motor_set_speed(speed);

		} else if (!reached_home) {
			// move back to starting point and terminate thread
			speed = move_home(&reached_home);
			left_motor_set_speed(speed);
			right_motor_set_speed(speed);

		} else {
			// calculate and set initial distance
			draw_reset();
			set_init_length(true_diff_length, expected_diff_length);
			chThdTerminate(ptr_calibrate);
		}

		chThdSleepMilliseconds(CALIBRATION_PERIOD);

	}

	is_calibrating = false;
	// return true length difference on exit
	chThdExit((msg_t)true_diff_length);
}



/**
 * @brief      Set home point (top middle of canvas)
 * @note       Top center coordinates of canvas is (X_RESOLUTION/2, 0)
 */
static THD_WORKING_AREA(wa_set_home, 256);
static THD_FUNCTION(thd_set_home, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while (!chThdShouldTerminateX()) {

		chThdSleepMilliseconds(SET_HOME_PERIOD);
		// go up
		if ( get_calibrated_prox(IR8) > IR_THRESHOLD
			|| get_calibrated_prox(IR1) > IR_THRESHOLD) {
			left_motor_set_speed(CALIBRATION_SPEED);
			right_motor_set_speed(CALIBRATION_SPEED);

		// go down
		} else if ( get_calibrated_prox(IR5) > IR_THRESHOLD) {
			left_motor_set_speed(-CALIBRATION_SPEED);
			right_motor_set_speed(-CALIBRATION_SPEED);

		// go left
		} else if (get_calibrated_prox(IR7) > IR_THRESHOLD) {
			left_motor_set_speed(-CALIBRATION_SPEED);
			right_motor_set_speed(CALIBRATION_SPEED);

		// go right
		} else if (get_calibrated_prox(IR2) > IR_THRESHOLD) {
			left_motor_set_speed(CALIBRATION_SPEED);
			right_motor_set_speed(-CALIBRATION_SPEED);

		} else {
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}

		chThdSleepMilliseconds(SET_HOME_PERIOD);
	}
	is_setting_home = false;
	chThdExit(0);
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
		ptr_calibrate = chThdCreateStatic(wa_calibrate, sizeof(wa_calibrate),
		                                  NORMALPRIO+1, thd_calibrate, NULL);
		is_calibrating = true;
	}
}

uint16_t cal_stop_thd(void)
{
	if (is_calibrating) {

		is_calibrating = false;
		chThdTerminate(ptr_calibrate);
		// send message to unblock chMsgWait()
		if (is_waiting)
			(void)chMsgSend(ptr_calibrate, 0);
		if (is_waiting_color)
			chBSemSignal(&sem_changed_color);

		uint16_t true_diff_length = chThdWait(ptr_calibrate);

		// return true length difference (steps)
		return true_diff_length;
	}
	return 0;
}

bool cal_get_state(void)
{
	return is_calibrating;
}

void cal_signal_changed_colors(void)
{
	if (is_waiting_color)
		chBSemSignal(&sem_changed_color);
}

void cal_create_home_thd(void)
{
	if (!is_setting_home) {
		ptr_set_home = chThdCreateStatic(wa_set_home, sizeof(wa_set_home),
		                                  NORMALPRIO, thd_set_home, NULL);
		is_setting_home = true;
	}
}

void cal_stop_home_thd(void)
{
	if (is_setting_home) {
		chThdTerminate(ptr_set_home);
		draw_reset();
		is_setting_home = false;
	}
}

bool cal_get_home_state(void)
{
	return is_setting_home;
}
