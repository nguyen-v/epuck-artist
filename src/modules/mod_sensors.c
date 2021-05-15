/**
 * @file    mod_sensors.c
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
#include "msgbus/messagebus.h"
#include "chprintf.h"
#include <usbcfg.h>

// e-puck 2 main processor headers

#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/proximity.h"

// Module headers

#include <mod_sensors.h>
#include <mod_communication.h>
#include <mod_calibration.h>
#include <mod_data.h>
#include <mod_state.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define BLINK_PERIOD       200
#define BLINK_MAX_COUNT    3
#define TOF_PERIOD         100

#define OBSERVED_NOISE_COVARIANCE 6.5f

/*===========================================================================*/
/* Bus related declarations.                                                 */
/*===========================================================================*/

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static uint16_t dist_mm_kalman = 0;

/*===========================================================================*/
/* Module thread pointers.                                                   */
/*===========================================================================*/

static thread_t* ptr_tof_kalman;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/


/**
 * @brief               1D Kalman filter
 *
 * @return              filtered value
 * @note                Kalman filter is defined in VL53L0X.c because we want
 *                      it to be ready (i.e. running) alongside the thread
 *                      defined in the same file (because values need to be
 *                      stable when accessed).
 *                      sources: https://www.youtube.com/watch?v=ruB917YmtgE
 *                               https://en.wikipedia.org/wiki/Kalman_filter
 */
static uint16_t kalman1d(uint16_t U)
{
	static const float R = OBSERVED_NOISE_COVARIANCE;
	static const float H = 1.00;       // observation model
	static float Q = 1.0;              // initial estimated covariance
	static float P = 0;                // initial error covariance
	static uint16_t U_hat = 0;         // initial predicted state
	static float K = 0;

	K = P*H/(H*P*H+R);                 // calculate Kalman gain
	U_hat = U_hat + K*(U-H*U_hat);     // updated state estimate
	P = (1-K*H)*P +Q;                  // updated estimate covariance
	return U_hat;
}



/*===========================================================================*/
/* Module threads.                                                           */
/*===========================================================================*/

/**
 * @brief   Thread for applying Kalman filter to TOF measurements.
 */
static THD_WORKING_AREA(wa_tof_kalman, 256);
static THD_FUNCTION(thd_tof_kalman, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while (1) {
		dist_mm_kalman = kalman1d(VL53L0X_get_dist_mm());
//		chprintf((BaseSequentialStream *)&SDU1, "dist %d", dist_mm_kalman);
		chThdSleepMilliseconds(TOF_PERIOD);
	}
}

/**
 * @brief            Create TOF measurement thread with Kalman filter
 * @return           none
 */
static void tof_kalman_create_thd(void)
{
	ptr_tof_kalman = chThdCreateStatic(wa_tof_kalman, sizeof(wa_tof_kalman),
	                                   NORMALPRIO, thd_tof_kalman, NULL);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void sensors_init(void)
{
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	proximity_start();
	VL53L0X_start();
	tof_kalman_create_thd();
	calibrate_ir();
}


uint16_t sensors_tof_kalman(void)
{
	return dist_mm_kalman;
}


uint16_t sensors_tof_wait(uint16_t distance_min, uint16_t distance_max,
                          uint8_t distance_threshold, uint16_t time_ms)
{
	uint8_t state = 0;
	uint16_t time_interval = time_ms/4;
	uint16_t current_dist = 0;
	uint16_t prev_dist = current_dist;

	while (state != 4 && cal_get_state() == true) {
		current_dist = dist_mm_kalman;
		if (state == 0) {
			// turn off the leds
			palSetPad(GPIOD, GPIOD_LED1);
			palSetPad(GPIOD, GPIOD_LED3);
			palSetPad(GPIOD, GPIOD_LED5);
			palSetPad(GPIOD, GPIOD_LED7);
		}

		if (state == 0 && distance_min <= current_dist && distance_max >= current_dist) {
			state = 1;
			prev_dist = current_dist;
			palClearPad(GPIOD, GPIOD_LED1);
			chThdSleepMilliseconds(time_interval);
			current_dist = dist_mm_kalman;
		} else {
			state = 0;
		}

		if (state == 1 && abs((int16_t)current_dist - (int16_t)prev_dist)
		                                          <= distance_threshold) {
			state = 2;
			prev_dist = current_dist;
			palClearPad(GPIOD, GPIOD_LED3);
			chThdSleepMilliseconds(time_interval);
			current_dist = dist_mm_kalman;
		} else {
			state = 0;
		}

		if (state == 2 && abs((int16_t)current_dist - (int16_t)prev_dist)
		                                          <= distance_threshold) {
			state = 3;
			prev_dist = current_dist;
			palClearPad(GPIOD, GPIOD_LED5);
			chThdSleepMilliseconds(time_interval);
			current_dist = dist_mm_kalman;
		} else {
			state = 0;
		}

		if (state == 3 && abs((int16_t)current_dist - (int16_t)prev_dist)
		                                          <= distance_threshold) {
			state = 4;
			prev_dist = current_dist;
			palClearPad(GPIOD, GPIOD_LED7);
			chThdSleepMilliseconds(time_interval);
		} else {
			state = 0;
		}
		chThdSleepMilliseconds(TOF_PERIOD);
	}

	// blink leds on success
	for (uint8_t i = 0; i < BLINK_MAX_COUNT; ++ i) {

		palClearPad(GPIOD, GPIOD_LED1);
		palClearPad(GPIOD, GPIOD_LED3);
		palClearPad(GPIOD, GPIOD_LED5);
		palClearPad(GPIOD, GPIOD_LED7);

		chThdSleepMilliseconds(BLINK_PERIOD);

		palSetPad(GPIOD, GPIOD_LED1);
		palSetPad(GPIOD, GPIOD_LED3);
		palSetPad(GPIOD, GPIOD_LED5);
		palSetPad(GPIOD, GPIOD_LED7);

		chThdSleepMilliseconds(BLINK_PERIOD);
	}
	return current_dist;
}
