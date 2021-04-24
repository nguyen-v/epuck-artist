/**
 * @file    mod_sensors.c
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

#include <mod_sensors.h>
#include <mod_communication.h>
#include <mod_data.h>
#include <mod_state.h>

// test
#include <motors.h>
#include "arm_math.h"
#include <mod_draw.h>
#include "msgbus/messagebus.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/proximity.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//static void sensors_start_proximity(void)
//{
//	proximity_start();
//}
//
//static void sensors_start_tof(void)
//{
//	VL53L0X_start();
//}




void sensors_init(void)
{
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	proximity_start();
	VL53L0X_start();
}

uint16_t sensors_tof_kalman(void)
{
	return VL53L0X_get_dist_mm_kalman();
}

uint16_t sensors_tof_wait(uint16_t distance_min, uint16_t distance_max,
						uint8_t distance_threshold, uint16_t time_ms)
{
	uint8_t state = 0;
	uint16_t time_interval = time_ms/4;
	uint16_t current_dist = 0;
	uint16_t prev_dist = current_dist;

	while (state != 4) {
		current_dist = sensors_tof_kalman();
		if(state == 0) {
			// turn off the leds
			palSetPad(GPIOD, GPIOD_LED1);
			palSetPad(GPIOD, GPIOD_LED3);
			palSetPad(GPIOD, GPIOD_LED5);
			palSetPad(GPIOD, GPIOD_LED7);
		}

		if(state == 0 && distance_min <= current_dist && distance_max >= current_dist) {
			state = 1;
			chprintf((BaseSequentialStream *)&SDU1, "STATE1 %d \r \n", current_dist);
			prev_dist = current_dist;
			palClearPad(GPIOD, GPIOD_LED1);
			chThdSleepMilliseconds(time_interval);
			current_dist = sensors_tof_kalman();
		} else {
			state = 0;
		}
		if(state == 1 && abs((int16_t)current_dist - (int16_t)prev_dist) <= distance_threshold) {
			state = 2;
			chprintf((BaseSequentialStream *)&SDU1, "STATE2 %d \r \n", current_dist);
			prev_dist = current_dist;
			palClearPad(GPIOD, GPIOD_LED3);
			chThdSleepMilliseconds(time_interval);
			current_dist = sensors_tof_kalman();
		} else {
			state = 0;
		}
		if(state == 2 && abs((int16_t)current_dist - (int16_t)prev_dist) <= distance_threshold) {
			state = 3;
			chprintf((BaseSequentialStream *)&SDU1, "STATE3 %d \r \n", current_dist);
			prev_dist = current_dist;
			palClearPad(GPIOD, GPIOD_LED5);
			chThdSleepMilliseconds(time_interval);
			current_dist = sensors_tof_kalman();
		} else {
			state = 0;
		}
		if(state == 3 && abs((int16_t)current_dist - (int16_t)prev_dist) <= distance_threshold) {
			state = 4;
			chprintf((BaseSequentialStream *)&SDU1, "STATE4 %d \r \n", current_dist);
			prev_dist = current_dist;
			palClearPad(GPIOD, GPIOD_LED7);
			chThdSleepMilliseconds(time_interval);
		} else {
			state = 0;
		}

	}
	for(uint8_t i = 0; i < 3; ++ i) {
		palClearPad(GPIOD, GPIOD_LED1);
		palClearPad(GPIOD, GPIOD_LED3);
		palClearPad(GPIOD, GPIOD_LED5);
		palClearPad(GPIOD, GPIOD_LED7);
		chThdSleepMilliseconds(200);
		palSetPad(GPIOD, GPIOD_LED1);
		palSetPad(GPIOD, GPIOD_LED3);
		palSetPad(GPIOD, GPIOD_LED5);
		palSetPad(GPIOD, GPIOD_LED7);
		chThdSleepMilliseconds(200);
	}
	return current_dist;
}
