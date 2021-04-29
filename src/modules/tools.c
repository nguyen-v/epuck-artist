/*
 * tools.c
 *
 *  Created on: 12 avr. 2021
 *      Author: YassineBakkali
 */
#include <math.h>
#include <stdint.h>

#include <tools.h>



float perpendicular_distance(struct cartesian_coord start, struct cartesian_coord end, struct cartesian_coord point){

	int32_t line1 = start.x - end.x;
	int32_t line2 = start.y - end.y;

	int32_t vec_x = point.x - start.x;
	int32_t vec_y = point.y - start.y;

	float cross_prod_1 = (line1*vec_y)-(line2*vec_x);
	float dot_prod = line1*line1 + line2*line2;

	float distance =  fabs(cross_prod_1) / sqrt(dot_prod);

return distance;
}



float two_point_distance(struct cartesian_coord point1, struct cartesian_coord point2){

	float distance = 0;
	int32_t  a = point1.x - point2.x ;
	int32_t  b = point1.y - point2.y ;
	distance = sqrt(a*a + b*b);
	return distance;
}

