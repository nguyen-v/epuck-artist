/*
 * mod_path.h
 *
 *  Created on: 7 avr. 2021
 *      Author: YassineBakkali
 */

#ifndef _MOD_PATH_H_
#define _MOD_PATH_H_

#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <camera/po8030.h>
#include <mod_img_processing.h>
#include <mod_data.h>
#include <tools.h>



struct edge_pos{
	struct cartesian_coord pos;
	uint16_t index;
};

enum px_status{line = 0, edge = 1};

enum edge_status{start = 0, end = 1, init = 2};

struct edge_track{

	struct cartesian_coord pos;
	uint8_t label;
	enum px_status start_end;	//line = 0, edge = 1;
	enum colour color;

};



void path_planning(uint8_t *img_buffer, enum colour *color);
uint8_t edge_scanning(uint8_t *img_buffer, uint16_t *count_lab);
void path_labelling(uint8_t *img_buffer);
void edge_tracing(struct edge_track *contours, struct cartesian_coord *edges, enum colour *color);
uint16_t path_optimization(struct edge_track *contours, uint16_t size, struct edge_track* dest, uint16_t destlen);
void nearest_neighbour(struct cartesian_coord *edges, uint16_t *edge_index, enum edge_status *status);
void img_resize(struct cartesian_coord* path, uint16_t path_size);

void flatten(uint16_t *count_lab, uint16_t count);
void save_pos(struct edge_track *pos, uint8_t x, uint8_t y, uint8_t label, enum px_status start_end, enum colour color, uint8_t k);
void save_edge_pos(struct cartesian_coord *pos, uint8_t x, uint8_t y, uint8_t curve_index, uint8_t this_index);
void swap(uint16_t *edges1, uint16_t *edges2);
uint16_t position(uint8_t pos_x, uint8_t pos_y);
uint16_t merge(uint16_t *count_lab, uint16_t x, uint16_t y);

#endif /* _MOD_PATH_H_ */

