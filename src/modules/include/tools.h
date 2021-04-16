/*
 * tools.h
 *
 *  Created on: 12 avr. 2021
 *      Author: YassineBakkali
 */

#ifndef _TOOLS_H_
#define _TOOLS_H_

struct px_pos{

	uint8_t pos_x;
	uint8_t pos_y;

};

uint8_t flatten(uint8_t old_img);
void push_back(uint8_t temp_img_val, uint8_t new_img);
uint8_t merge(uint8_t count_lab, uint8_t x, uint8_t y);
uint16_t position(uint8_t pox_x, uint8_t pos_y);
float perpendicular_distance(struct px_pos start, struct px_pos end, struct px_pos point);
float two_point_distance(struct px_pos point1, struct px_pos point2);

#endif /* SRC_MODULES_INCLUDE_TOOLS_H_ */
