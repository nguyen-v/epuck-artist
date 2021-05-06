/**
 * @file    mod_draw.c
 * @brief   Module for moving the robot to XY coordinates.
 * @note
 */

// C standard header files

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

// ChibiOS headers

#include "hal.h"
#include "ch.h"
#include <usbcfg.h>
#include "chprintf.h"
#include "chschd.h"

// e-puck 2 main processor headers

#include <motors.h>

// Module headers

#include <mod_draw.h>
#include <mod_communication.h>
#include <mod_data.h>
#include <def_epuck_field.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define MAX_SPEED              250    // steps/s

#define STEP_THRESHOLD         5      // defines how close we should get to
								       // goal length in steps

#define TIME_SLEEP_MIN         20     // ms motors dont have time to react if
                                       // too low
#define DEFAULT_HEIGHT         100.0f // cm

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static uint16_t y0_st = DEFAULT_HEIGHT*CM_TO_STEP;
static uint16_t x0_st = (SUPPORT_DISTANCE_ST-SPOOL_DISTANCE_ST)/2.;
static uint16_t len0_st = 0;

static bool is_drawing = false;
static bool is_paused = false;

/*===========================================================================*/
/* Module thread pointers.                                                   */
/*===========================================================================*/

static thread_t* ptr_draw;

/*===========================================================================*/
/* Module local functions.                                                	 */
/*===========================================================================*/

/**
 * @brief                    Moves the robot by a certain amount of steps
 * @param[in]   step_left    Number of steps to be traveled by left motor
 * @param[in]   step_right   Number of steps to be traveled by right motor
 * @param[in]   speed        Maximum speed in step/s
 * @return                   none
 */
static void motor_set_step(int32_t step_left, int32_t step_right, uint16_t speed)
{
	// calculate absolute values for computation of sign and sleep time.
	uint16_t step_left_abs = abs(step_left);
	uint16_t step_right_abs = abs(step_right);
	uint32_t time = 0;
	float speed_left = 0;
	float speed_right = 0;

	if (step_left_abs > step_right_abs) {
		time = 1000*step_left_abs/speed; // ms
		speed_left = speed*step_left/step_left_abs;
		speed_right = speed * ((float)step_right/step_left_abs);
	} else {
		time = 1000*step_right_abs/speed; // ms
		speed_left = speed * ((float)step_left/step_right_abs);
		speed_right = speed*step_right/step_right_abs;
	}

	// minus sign because of the orientation of e-puck
	right_motor_set_speed(-speed_left);
	left_motor_set_speed(-speed_right);
	chThdSleepMilliseconds(time);
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

/**
 * @brief                    Offsets x position to match drawing area
 * @param[in]   x            x coordinate
 * @return                   x coordinate with offset
 */
static uint16_t offset_x_pos(uint16_t x)
{
	return x + (X_RESOLUTION - IM_MAX_WIDTH)/2;
}

/*===========================================================================*/
/* Module threads.                                                   		 */
/*===========================================================================*/

/**
 * @brief   Thread for drawing a picture from information contained in
 *          position and color buffers (mod_data).
 *
 */
static THD_WORKING_AREA(wa_draw, 1024);
static THD_FUNCTION(thd_draw, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	uint16_t i = 0;
	uint16_t length = data_get_length();

	cartesian_coord* pos = data_get_pos();
	uint8_t* color = data_get_color();
	uint8_t prev_color = white;

	for(i = 0; i < length && !chThdShouldTerminateX(); ++i) {

		if (color[i] != prev_color) {
			com_request_color(color[i]);
			prev_color = color[i];
			draw_pause_thd();
		}

		chSysLock();
		if (is_paused) {
		  chSchGoSleepS(CH_STATE_SUSPENDED);
		}
		chSysUnlock();

		draw_move(offset_x_pos(pos[i].x), pos[i].y);
	}

	// reset stepper position and lift pen when drawing is complete
	com_request_color(none);

	is_drawing = false;
	chThdExit(0);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void draw_reset(void)
{
	right_motor_set_speed(0);
	left_motor_set_speed(0);
	right_motor_set_pos(0);
	left_motor_set_pos(0);
}


void draw_create_thd(void)
{
	if (!is_drawing) {
		ptr_draw = chThdCreateStatic(wa_draw, sizeof(wa_draw), NORMALPRIO,
		                             thd_draw, NULL);
		is_drawing = true;
	}
}

void draw_stop_thd(void)
{
	if(is_drawing) {
		chThdTerminate(ptr_draw);
		is_drawing = false;
		is_paused = false;
	}
}

void draw_pause_thd(void)
{
	if(is_drawing)
		is_paused = true;
}

void draw_resume_thd(void)
{
	chSysLock();
	if (is_drawing && is_paused) {
	  chSchWakeupS(ptr_draw, CH_STATE_READY);
	  is_paused = false;
	}
	chSysUnlock();
}

bool draw_get_state(void)
{
	return is_drawing;
}

void draw_set_init_length(float y_length)
{
	y0_st = CM_TO_STEP*y_length;
	len0_st = sqrt(x0_st*x0_st + y0_st*y0_st);
}

uint16_t draw_get_length_av_current(void)
{
	uint16_t len_l_current = len0_st - right_motor_get_pos();
	uint16_t len_r_current = len0_st - left_motor_get_pos();
	return (len_l_current+len_r_current)/2;
}

uint16_t draw_get_length_av_next(uint16_t x, uint16_t y)
{
	uint16_t x_st = x*CART_TO_ST;
	uint16_t y_st = y*CART_TO_ST + y0_st;
	uint16_t x_l_st = x_st + MARGIN_ST - SPOOL_DISTANCE_ST/2;
	uint16_t x_r_st = SUPPORT_DISTANCE_ST - SPOOL_DISTANCE_ST/2 - MARGIN_ST - x_st;

	// calculate next length
	uint16_t len_l = sqrt(x_l_st*x_l_st + y_st*y_st);
	uint16_t len_r = sqrt(x_r_st*x_r_st + y_st*y_st);
	return (len_r+len_l)/2;
}

void draw_move(uint16_t x, uint16_t y)
{
	// get current wire lengths in steps
	// Increase in step -> Decrease in wire length (because of e-puck orientation)
	// Left motor in charge of the right wire and right motor in charge of the left wire
	uint16_t len_l_current = len0_st - right_motor_get_pos();
	uint16_t len_r_current = len0_st - left_motor_get_pos();

	// calculate coordinates relative to top-left attach point
	uint16_t x_st = x*CART_TO_ST;
	uint16_t y_st = y*CART_TO_ST + y0_st;
	uint16_t x_l_st = x_st + MARGIN_ST - SPOOL_DISTANCE_ST/2;
	uint16_t x_r_st = SUPPORT_DISTANCE_ST - SPOOL_DISTANCE_ST/2 - MARGIN_ST - x_st;

	// calculate next length
	uint16_t len_l = sqrt(x_l_st*x_l_st + y_st*y_st);
	uint16_t len_r = sqrt(x_r_st*x_r_st + y_st*y_st);

	// move until next length
	while (abs(len_l-len_l_current)>STEP_THRESHOLD || abs(len_r-len_r_current)>STEP_THRESHOLD) {
		motor_set_step(len_l-len_l_current, len_r-len_r_current, MAX_SPEED);
		len_l_current = len0_st - right_motor_get_pos();
		len_r_current = len0_st - left_motor_get_pos();
	}
}

