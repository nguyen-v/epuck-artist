/**
 * @file    mod_data.h
 * @brief   Header for drawing module.
 */

#ifndef _MOD_DRAW_H_
#define _MOD_DRAW_H_

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/



/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

//void right_motor_set_length(uint16_t step, int16_t speed);
//
//void left_motor_set_length(uint16_t step, int16_t speed);

void draw_set_init_length(float y_length);
void draw_reset(void);


/**
 * @brief			Moves the e-puck to specified XY coordinates.
 * @note			(0,0) is at the top-left of the canvas.
 * @param[in] x 	x coordinate, ranging from (0-X_RESOLUTION)
 * @param[in] y 	y coordinate, ranging from ?
 * @return			none
 */
void draw_move(uint16_t x, uint16_t y);
//void motor_set_step(int32_t step_left, int32_t step_right, uint16_t speed);

#endif
