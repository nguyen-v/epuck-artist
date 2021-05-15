/**
 * @file    tools.h
 * @brief   External declarations of tools.c
 */

#ifndef _TOOLS_H_
#define _TOOLS_H_

// Module headers
#include <mod_data.h>

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/**
 * @brief               calculates pixel position of 1D image buffer
 * @param[in]   x       x coordinate in range [0, IM_LENGTH_PX]
 * @param[in]   y       y coordinate in range [0, IM_HEIGHT_PX]
 * @return              corresponding 1D buffer position
 */
uint16_t position(uint8_t pos_x, uint8_t pos_y);

/**
 * @brief               calculates the perpendicular distance between a starting coordinate
 *                      and end coordinate and an intermediate coordinate
 * @param[in]   start   starting point
 * @param[in]   end     ending point
 * @param[in]   point   intermediate point
 * @return              perpendicular distance in px
 */
float perpendicular_distance(struct cartesian_coord start, struct cartesian_coord end,
                             struct cartesian_coord point);

/**
 * @brief               calculates the distance between 2 points
 * @param[in]   point1  starting point
 * @param[in]   point2  ending point
 * @return              distance between the 2 points in px
 */
float two_point_distance(struct cartesian_coord point1, struct cartesian_coord point2);

#endif /* _TOOLS_H_ */
