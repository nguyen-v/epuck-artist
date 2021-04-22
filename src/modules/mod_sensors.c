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



uint16_t sensors_tof_kalman(void)
{
	return VL53L0X_get_dist_mm_kalman();
}

void sensors_init(void)
{
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	proximity_start();
	VL53L0X_start();
}


