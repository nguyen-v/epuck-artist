/*
 * mod_path.h
 *
 *  Created on: 7 avr. 2021
 *      Author: YassineBakkali
 */

#ifndef _MOD_PATH_H_
#define _MOD_PATH_H_

typedef struct edge_track{

	uint8_t pos_x;
	uint8_t pos_y;
	uint16_t label;
	px_status start_end;	//start = 0, end = 1;


};

enum px_status{line = 0, edge = 1};


uint8_t edge_scanning(uint8_t img_buffer, uint8_t count_lab);
edge_track path_labelling(uint8_t img_buffer);
void path_planning(void);
void path_optimization(void);
void nearest_neighbour(void);

uint8_t flatten(uint8_t old_img);
uint8_t push_back(uint8_t temp_img_val, uint8_t new_img);
uint8_t merge(uint8_t count_lab, uint8_t x, uint8_t y);
uint16_t position(uint8_t pox_x, uint8_t pos_y);

#endif /* _MOD_PATH_H_ */
