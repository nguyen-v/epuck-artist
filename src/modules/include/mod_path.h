/*
 * mod_path.h
 *
 *  Created on: 7 avr. 2021
 *      Author: YassineBakkali
 */

#ifndef _MOD_PATH_H_
#define _MOD_PATH_H_

#include <camera/po8030.h>
#include <image_processing.h>
#include <include/tools.h>
#include <stdio.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

struct edge_pos{

	struct px_pos pos;
	uint16_t index;
};

enum px_status{line = 0, edge = 1};

enum edge_status{start = 0, end = 1};

struct edge_track{

	struct px_pos pos;
	uint8_t label;
	enum px_status start_end;	//line = 0, edge = 1;

};

struct path_motor{

	struct px_pos pos;
	enum colour color;

};





struct path_motor path_planning(uint8_t *img_buffer);
void edge_scanning(uint8_t *img_buffer, uint8_t *count_lab);
void path_labelling(uint8_t *img_buffer);
void edge_tracing(struct edge_track *contours, struct edge_pos *edges);
uint16_t path_optimization(struct edge_track *contours, uint16_t size, struct edge_track* dest, uint16_t destlen);
void nearest_neighbour(struct edge_pos *edges, uint16_t *edge_index);

void save_pos(struct edge_track *pos, uint8_t x, uint8_t y, uint8_t label, enum px_status start_end, uint8_t k);
void save_edge_pos(struct edge_pos *pos, uint8_t x, uint8_t y, uint8_t curve_index, uint8_t this_index);
void swap(struct px_pos *edges1, struct px_pos *edges2);

#endif /* _MOD_PATH_H_ */
