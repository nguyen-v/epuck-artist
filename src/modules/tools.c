/**
 * @file    tools.c
 * @brief   Definitions of useful distance calculation and position functions.
 */


// C standard header files

#include <math.h>
#include <stdint.h>

// Module headers

#include <tools.h>
#include <mod_img_processing.h>

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

uint16_t position(uint8_t pos_x, uint8_t pos_y){

	uint16_t position = pos_x + (uint16_t)pos_y*IM_LENGTH_PX;
	return position;
}

float perpendicular_distance(struct cartesian_coord start, struct cartesian_coord end, struct cartesian_coord point){

	int32_t line1 = start.x - end.x; // int32 to avoid underflow
	int32_t line2 = start.y - end.y;

	int32_t vec_x = point.x - start.x;
	int32_t vec_y = point.y - start.y;

	float cross_prod_1 = (line1*vec_y)-(line2*vec_x);
	float dot_prod = line1*line1 + line2*line2;

	float distance =  fabs(cross_prod_1) / sqrtf(dot_prod);

return distance;
}


float two_point_distance(struct cartesian_coord point1, struct cartesian_coord point2){

	float distance = 0;
	int32_t  a = point1.x - point2.x ;
	int32_t  b = point1.y - point2.y ;
	distance = sqrtf(a*a + b*b);
	return distance;
}

