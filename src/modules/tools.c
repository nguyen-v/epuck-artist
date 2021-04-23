/*
 * tools.c
 *
 *  Created on: 12 avr. 2021
 *      Author: YassineBakkali
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <tools.h>



float perpendicular_distance(struct cartesian_coord start, struct cartesian_coord end, struct cartesian_coord point){

	uint16_t line1 = start.x - end.x;
	uint16_t line2 = start.y - end.y;

	uint16_t vec_x = point.x - start.x;
	uint16_t vec_y = point.y - start.y;

	float cross_prod_1 = (line1*vec_y)-(line2*vec_x);
	float dot_prod = line1*line1 + line2*line2;

	float distance =  abs(cross_prod_1) / sqrt(dot_prod);

return distance;
}



float two_point_distance(struct cartesian_coord point1, struct cartesian_coord point2){

	float distance = 0;
	uint16_t  a = point1.x - point2.x ;
	uint16_t  b = point1.y - point2.y ;
	distance = sqrt(a*a + b*b);
	return distance;
}
