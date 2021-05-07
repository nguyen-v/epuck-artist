/**
 * @file    mod_data.h
 * @brief   External declarations of drawing module.
 */

#ifndef _MOD_DRAW_H_
#define _MOD_DRAW_H_

/*===========================================================================*/
/* Exported constants                                                        */
/*===========================================================================*/

#define IM_MAX_HEIGHT     150 // px
#define IM_MAX_WIDTH      150 // px

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/**
 * @brief            Stops stepper motor and resets position
 * @return           none
 */
void draw_reset(void);

/**
 * @brief            Create drawing thread
 * @return           none
 */
void draw_create_thd(void);

/**
 * @brief            Stop drawing thread
 * @return           none
 */
void draw_stop_thd(void);

/**
 * @brief            Pause drawing thread
 * @return           none
 */
void draw_pause_thd(void);

/**
 * @brief            Resume drawing thread
 * @return           none
 */
void draw_resume_thd(void);

/**
 * @brief            Signals that color has changed
 * @return           none
 */
void draw_signal_changed_colors(void);

/**
 * @brief            Returns current thread state
 * @return           True if currently drawing, false otherwise
 */
bool draw_get_state(void);

/**
 * @brief            Set initial distance between e-puck and the line between
 *                   2 thread attach points
 * @return           none
 */
void draw_set_init_length(float y_length);

/**
 * @brief            Get the average current wire length in steps
 * @return           none
 */
uint16_t draw_get_length_av_current(void);

/**
 * @brief            Get the average length needed to reach xy coordinates
 * @param[in]   x    x coordinate in pixels
 * @param[in]   y    y coordinate in pixels
 * @return           Average length in steps
 */
uint16_t draw_get_length_av_next(uint16_t x, uint16_t y);

/**
 * @brief            Moves the e-puck to specified XY coordinates
 * @note             (0,0) is at the top-left of the canvas
 * @param[in]   x    x coordinate in pixels
 * @param[in]   y    y coordinate in pixels
 * @return           Average length in steps to reach desired coordinates
 */
void draw_move(uint16_t x, uint16_t y);

#endif /* _MOD_DRAW_H_ */
