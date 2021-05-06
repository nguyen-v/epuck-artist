/**
 * @file    mod_calibration.h
 * @brief   External declarations of calibration module.
 */

#ifndef _MOD_CALIBRATION_H_
#define _MOD_CALIBRATION_H_

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/**
 * @brief                        Creates calibration thread
 * @return                       none
 */
void cal_create_thd(void);

/**
 * @brief                        Stops calibration thread
 * @return                       True calibration distance in steps
 */
uint16_t cal_stop_thd(void);

/**
 * @brief                        Sets goal calibration distance
 * @return                       none
 */
void cal_set_goal_distance(void);

/**
 * @brief                        Returns current calibration state
 * @return                       True if currently calibrating, false otherwise
 */
bool cal_get_state(void);

/**
 * @brief                        Signals that color has changed
 * @return                       none
 */
void cal_signal_changed_colors(void);

#endif /* _MOD_CALIBRATION_H_ */
