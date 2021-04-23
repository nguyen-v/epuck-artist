/*
 * tools.h
 *
 *  Created on: 12 avr. 2021
 *      Author: YassineBakkali
 */

#ifndef _TOOLS_H_
#define _TOOLS_H_

#include <mod_data.h>

float perpendicular_distance(struct cartesian_coord start, struct cartesian_coord end, struct cartesian_coord point);
float two_point_distance(struct cartesian_coord point1, struct cartesian_coord point2);

#endif /* _TOOLS_H_ */
