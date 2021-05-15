/**
 * @file    mod_sensors.h
 * @brief   External declarations for sensor module.
 */

#ifndef _MOD_SENSORS_H_
#define _MOD_SENSORS_H_

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/**
 * @brief               initializes TOF and proximity sensor threads
 * @return              none
 */
void sensors_init(void);

/**
 * @brief               returns kalman filtered data for TOF sensor
 * @return              distance from TOF sensor in mm
 * @note                Kalman filter is defined in VL53L0X.c because we want
 *                      it to be ready (i.e. running) alongside the thread
 *                      defined in the same file (because filter needs to be
 *                      stable when we call this function).
 */
uint16_t sensors_tof_kalman(void);

/**
 * @brief               waits for an object to be in range (distance_min,
 *                      distance_max) for a period of time_ms, with the object
 *                      being stable in a range given by distance_threshold
 * @return              position in mm for which the object is stable
 */
uint16_t sensors_tof_wait(uint16_t distance_min, uint16_t distance_max,
                          uint8_t distance_threshold, uint16_t time_ms);

#endif /* _MOD_SENSORS_H_ */
