/**
 * @file    mod_sensors.h
 * @brief   Header for drawing module.
 */

#ifndef _MOD_SENSORS_H_
#define _MOD_SENSORS_H_

void sensors_init(void);
uint16_t sensors_tof_kalman(void);
uint16_t sensors_tof_wait(uint16_t distance_min, uint16_t distance_max,
						uint8_t distance_threshold, uint16_t time_ms);

#endif
