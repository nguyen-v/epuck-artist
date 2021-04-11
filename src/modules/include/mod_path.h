/*
 * mod_path.h
 *
 *  Created on: 7 avr. 2021
 *      Author: YassineBakkali
 */

#ifndef _MOD_PATH_H_
#define _MOD_PATH_H_

typedef struct px_pos{

	uint8_t pos_x;
	uint8_t pos_y;

};


typedef struct edge_track{

	struct px_pos pos;
	uint8_t label;
	enum px_status start_end;	//line = 0, edge = 1;

};


enum px_status{line = 0, edge = 1};


uint8_t edge_scanning(uint8_t img_buffer, uint8_t count_lab);
struct edge_track path_labelling(uint8_t img_buffer);
void path_planning(void);
uint16_t path_optimization(void);
void nearest_neighbour(void);

uint8_t flatten(uint8_t old_img);
uint8_t push_back(uint8_t temp_img_val, uint8_t new_img);
uint8_t merge(uint8_t count_lab, uint8_t x, uint8_t y);
uint16_t position(uint8_t pox_x, uint8_t pos_y);
uint16_t perpendicular_distance(struct px_pos start, struct px_pos end, struct px_pos point);

void *array_concatenation(uint16_t *result, uint16_t *res1, uint16_t *res2, uint16_t size1, uint16_t size2);
void save_pos(edge_track *pos, uint8_t x, uint8_t y, uint8_t label,px_status start_end, uint8_t k);


#endif /* _MOD_PATH_H_ */
