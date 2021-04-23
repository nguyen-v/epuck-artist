/*
 * tools.c
 *
 *  Created on: 12 avr. 2021
 *      Author: YassineBakkali
 */
#include <tools.h>



uint8_t flatten(uint8_t *old_img){

	uint8_t temp_img[] = {0};
	if(sizeof(old_img) == 1)
		return old_img;
	uint8_t new_img[] = {0};
	for(uint8_t i=0; i<=sizeof(old_img); i++){
			temp_img = flatten(old_img(i));
			for(uint8_t j = 0; j<= sizeof(temp_img); ++j){
				push_back(temp_img[j], new_img);
			}
		}
	return new_img;
}

/**
 * @brief				A rewriting of the C++ function "push_back()"
 */
void push_back(uint8_t temp_img_val, uint8_t *new_img){

	uint8_t newer_img[sizeof(new_img)/sizeof(uint8_t) + 1] = {0};

	new_img = (uint_8*) realloc(new_img,sizeof(new_img)/sizeof(uint8_t) + 1);

	new_img[sizeof(new_img)/sizeof(uint8_t) + 1] = temp_img_val;
}

//n
/**
 * @brief		This function gives the same label to 2 pixels with different labels
 */
void merge(uint8_t *count_lab, uint8_t x, uint8_t y){

	while(count_lab[x]!=count_lab[y]){
		if(count_lab[x]>count_lab[y])
			count_lab[rootx]=count_lab[rooty];
		else
			count_lab[rooty]=count_lab[rootx];
	}
}


uint16_t position(uint8_t pox_x, uint8_t pos_y){

	uint16_t position = pos_x + pos_y*IM_LENGTH_PX;
	return position;

}


float perpendicular_distance(struct px_pos start, struct px_pos end, struct px_pos point){

	uint16_t line1 = start.pos_x - end.pos_x;
	uint16_t line2 = start.pos_y - end.pos_y;

	uint16_t vec_x = point.pos_x - start.pos_x;
	uint16_t vec_y = point.pos_y - start.pos_y;

	float cross_prod_1 = (line1*vec_y)-(line2*vec_x);
	float dot_prod = line1*line1 + line2*line2;

	float distance =  abs(cross_prod_1) / sqrt(dot_prod);

return distance;
}




float two_point_distance(struct px_pos point1, struct px_pos point2){

	float distance = 0;
	uint16_t  a = point1.pos_x - point2.pos_x ;
	uint16_t  b = point1.pos_y - point2.pos_y ;
	distance = sqrt(a*a + b*b);
	return distance;
}
