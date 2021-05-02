/**
 * @file    mod_path.c
 * @brief   Handles the capture and the processing of an image.
 */


/** ---------------------------------------------------------------------------
*
* Main buffer structure:
*
* -----------------------------------------------------------------------------
*
* contours: 	size: size_contours
* contains positions of each active pixel, ordered so that it follows a contour.
*
* -----------------------------------------------------------------------------
*
* edges: 		size: size_edges
* contains positions of extremities.
* Ordered by pairs, i.e. edges[0] goes with edges[1] and so on.
* note: Index is associated to the pixel position at the
* position <index> in the contours buffer.
*
* -----------------------------------------------------------------------------
*
* final_path:	size: total_size
* contains the final positions that the robot has to follow.
* note: final_path[0] contains initial robot position
*
* -----------------------------------------------------------------------------
*
* color:		size: total_size
* contains the color associated to each pixel in final_path.
*
* -----------------------------------------------------------------------------
*
* status:		size: size_edges
* contains edge status (start, end).
* note: it is separated from the edge_pos structure to avoid padding.
*
* -----------------------------------------------------------------------------
*/

// C standard header files
#include <stdlib.h>
#include <stdint.h>

// ChibiOS headers
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

// Module headers
#include <mod_path.h>
#include <tools.h>
#include <mod_img_processing.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define INIT_ROBPOS_PX 50
#define INIT_ROBPOS_PY 0

#define INIT_ARR 1

/** this dictates the maximum perpendicular distance in pixels between
 * approximated lines of optimized contour buffer and corresponding points of
 * the non optimized contour buffer.
 * Smaller values lead to more points for approximating a shape.
 */

#define MAX_PERP_DIST 0.75f

/** max allowed distance between two positions in the final buffer.
 * this parameter is important because the robot needs more than two points
 * to draw a straight line (mechanical constraint)
 */
#define MAX_PIXEL_DIST 3

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static edge_pos* edges;
static edge_track* contours;


static struct edge_track* res_out;
static struct edge_track* new_contour;

static uint8_t* status;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief				calculates pixel position of 1D image buffer
 * @param[in]	x		x coordinate in range [0, IM_LENGTH_PX]
 * @param[in]	y		y coordinate in range [0, IM_HEIGHT_PX]
 * @return				corresponding 1D buffer position
 */
static uint16_t position(uint8_t pos_x, uint8_t pos_y){

	uint16_t position = pos_x + (uint16_t)pos_y*IM_LENGTH_PX;
	return position;
}


/**
 * @brief						increases the size of the contours buffer and saves a new entry in it
 * @param[in] 	size			new size of the contour buffer
 * @param[in] 	new_edge		new contours value to pushback
 * @param[out] 	contours		pointer to contour buffer
 * @return						none
 */
static void pushback_contours(edge_track* contours, uint16_t size, edge_track new_contour){

	contours = realloc(contours, (size+1)*sizeof(edge_track));
	contours[size] = new_contour;

}


/**
 * @brief							Increases the size of the edges buffer and saves a new entry in it
 * @param[in] 	size				new size of the edges buffer
 * @param[in] 	new_edge			new edge value to pushback
 * @param[out]	edges				pointer to edges buffer
 * @return							none
 */
static void pushback_edges(edge_pos* edges, uint16_t size, edge_pos new_edge){

	edges = realloc(edges, (size+1)*sizeof(edge_pos));
	edges[size] = new_edge;

}




/**
 * @brief						fills contours and edges buffer with active pixels
 * @param[in]	img_buffer		pointer to image buffer
 * @param[out]	contours		pointer to contour buffer
 * @param[out]	edges			pointer to edges buffer
 * @return						none
 */
static void path_tracing(uint8_t* img_buffer, edge_track *contours, edge_pos *edges, uint16_t *size_contours, uint16_t *size_edges)
{
	// max has to be at IM_MAX_VALUE (because of output of canny_edge). Other values are chosen arbitrarily.
	enum px_status {max = IM_MAX_VALUE, visited = IM_MAX_VALUE-1, rewind = IM_MAX_VALUE-2, begin = IM_MAX_VALUE-3};
	const int8_t dx = 1;
	const int8_t dy = IM_LENGTH_PX;

	bool extremity_found = false;

	uint16_t edge_index = 0;

	edge_pos new_edge;
	edge_track new_contour;

	uint16_t x_temp = 1;
	uint16_t y_temp = 1;
	uint16_t pos = 0;
	for(uint8_t x = 1; x<IM_LENGTH_PX-1; ++x) {
		for(uint8_t y=1; y< IM_HEIGHT_PX-1; ++y) {
			pos = position(x,y);
			x_temp = x;
			y_temp = y;
			// Start from a pixel and move until extremity is found
			extremity_found = false;
			if(img_buffer[pos] == max) {

				img_buffer[pos] = visited;
				bool has_converged = false;
				while(!extremity_found) {
					// First check pixels with max intensity (not visited yet)
					// right
					if(img_buffer[pos+dx]==max) {
						pos += dx;
						++x_temp;
					}
					// left
					else if(img_buffer[pos-dx]==max) {
						pos -= dx;
						--x_temp;
					}
					// bottom
					else if(img_buffer[pos+dy]==max) {
						pos += dy;
						++y_temp;
					}
					// top
					else if(img_buffer[pos-dy]==max) {
						pos -= dy;
						--y_temp;
					}
					// bottom right
					else if(img_buffer[pos+dx+dy]==max) {
						pos += (dx+dy);
						++x_temp; ++y_temp;
					}
					// bottom left
					else if(img_buffer[pos-dx+dy]==max) {
						pos -= (dx-dy);
						--x_temp; ++y_temp;
					}
					// top right
					else if(img_buffer[pos+dx-dy]==max) {
						pos += (dx-dy);
						++x_temp; --y_temp;
					}
					// top left
					else if(img_buffer[pos-dx-dy]==max) {
						pos -= (dx+dy);
						--x_temp; --y_temp;
					}
					// Here we allow the robot to come back ONCE on a pixel that
					// has been marked as "rewind"
					else if(!has_converged && img_buffer[pos+dx]==rewind) {
						pos += dx;
						++x_temp;
						has_converged = true;
					}
					// left
					else if(!has_converged && img_buffer[pos-dx]==rewind) {
						pos -= dx;
						--x_temp;
						has_converged = true;
					}
					// bottom
					else if(!has_converged && img_buffer[pos+dy]==rewind) {
						pos += dy;
						++y_temp;
						has_converged = true;
					}
					// top
					else if(!has_converged && img_buffer[pos-dy]==rewind) {
						pos -= dy;
						--y_temp;
						has_converged = true;
					}
					// bottom right
					else if(!has_converged && img_buffer[pos+dx+dy]==rewind) {
						pos += (dx+dy);
						++x_temp; ++y_temp;
						has_converged = true;
					}
					// bottom left
					else if(!has_converged && img_buffer[pos-dx+dy]==rewind) {
						pos -= (dx-dy);
						--x_temp; ++y_temp;
						has_converged = true;
					}
					// top right
					else if(!has_converged && img_buffer[pos+dx-dy]==rewind) {
						pos += (dx-dy);
						++x_temp; --y_temp;
						has_converged = true;
					}
					// top left
					else if(!has_converged && img_buffer[pos-dx-dy]==rewind) {
						pos -= (dx+dy);
						--x_temp; --y_temp;
						has_converged = true;
					}
					else {
						extremity_found = true;

						new_edge.pos.x = x_temp;
						new_edge.pos.y = y_temp;
						new_edge.index = *size_contours;

						if(edge_index == 0)
							edges[0] = new_edge;
						else
							pushback_edges(edges, edge_index, new_edge);


						new_contour.pos.x = x_temp;
						new_contour.pos.y = y_temp;
						new_contour.start_end = 1;

						if(size_contours == 0)
							contours[0] = new_contour;
						else
							pushback_contours(contours, *size_contours, new_contour);


						++edge_index;
					}
					img_buffer[pos] = visited;
				}

				// Once extremity is found, rewind the path and find second extremity.
				img_buffer[pos] = begin; // this allows overlapping back on the starting position (e.g. closed curve)

				extremity_found = false;
				while(!extremity_found) {
					status = 0;
					// Check visited pixels first (to avoid rewinding the wrong pixels)

					// right
					if(img_buffer[pos+dx]==visited) {
						pos += dx;
						++x_temp;
					}
					// left
					else if(img_buffer[pos-dx]==visited) {
						pos -= dx;
						--x_temp;
					}
					// bottom
					else if(img_buffer[pos+dy]==visited) {
						pos += dy;
						++y_temp;
					}
					// top
					else if(img_buffer[pos-dy]==visited) {
						pos -= dy;
						--y_temp;
					}
					// bottom right
					else if(img_buffer[pos+dx+dy]==visited) {
						pos += (dx+dy);
						++x_temp; ++y_temp;
					}
					// bottom left
					else if(img_buffer[pos-dx+dy]==visited) {
						pos -= (dx-dy);
						--x_temp; ++y_temp;
					}
					// top right
					else if(img_buffer[pos+dx-dy]==visited) {
						pos += (dx-dy);
						++x_temp; --y_temp;
					}
					// top left
					else if(img_buffer[pos-dx-dy]==visited) {
						pos -= (dx+dy);
						--x_temp; --y_temp;
					}
					// Then, check max pixels or starting position.
					// Note: we don't want to connect back to starting position if line length is 2 px
					// size_contours-1 != edges[edge_index-1].index) handles this condition

					else if(img_buffer[pos+dx]==max || (img_buffer[pos+dx]==begin
							&& *size_contours-1 != edges[edge_index-1].index)) {
						pos += dx;
						++x_temp;
					}
					// left
					else if(img_buffer[pos-dx]==max || (img_buffer[pos-dx]==begin
							&& *size_contours-1 != edges[edge_index-1].index)) {
						pos -= dx;
						--x_temp;
					}
					// bottom
					else if(img_buffer[pos+dy]==max || (img_buffer[pos+dy]==begin
							&& *size_contours-1 != edges[edge_index-1].index)) {
						pos += dy;
						++y_temp;
					}
					// top
					else if(img_buffer[pos-dy]==max || (img_buffer[pos-dy]==begin
							&& *size_contours-1 != edges[edge_index-1].index)) {
						pos -= dy;
						--y_temp;
					}
					// bottom right
					else if(img_buffer[pos+dx+dy]==max || (img_buffer[pos+dx+dy]==begin
							&& *size_contours-1 != edges[edge_index-1].index)) {
						pos += (dx+dy);
						++x_temp; ++y_temp;
					}
					// bottom left
					else if(img_buffer[pos-dx+dy]==max || (img_buffer[pos-dx+dy]==begin
							&& *size_contours-1 != edges[edge_index-1].index)) {
						pos -= (dx-dy);
						--x_temp; ++y_temp;
					}
					// top right
					else if(img_buffer[pos+dx-dy]==max || (img_buffer[pos+dx-dy]==begin
							&& *size_contours-1 != edges[edge_index-1].index)) {
						pos += (dx-dy);
						++x_temp; --y_temp;
					}
					// top left
					else if(img_buffer[pos-dx-dy]==max || (img_buffer[pos-dx-dy]==begin
							&& *size_contours-1 != edges[edge_index-1].index)) {
						pos -= (dx+dy);
						--x_temp; --y_temp;
					}
					else {
						extremity_found = true;
						new_edge.pos.x = x_temp;
						new_edge.pos.y = y_temp;
						new_edge.index = *size_contours;
						new_contour.start_end = 1;
						pushback_edges(edges, edge_index, new_edge);

						++edge_index;
					}
					img_buffer[pos] = rewind;
					if(!extremity_found) {
						++(*size_contours);
						new_contour.start_end = 0;
					}
					new_contour.pos.x = x_temp;
					new_contour.pos.y = y_temp;
					pushback_contours(contours, *size_contours, new_contour);


				}
				++size_contours;
			}
		}
	}
	*size_edges = edge_index;
}




/**
 * @brief						optimizes the path, i.e. deletes redundant points between two edges
 * @param[in]	contour			pointer to buffer containing one contour
 * @param[out]	opt_contour		pointer to buffer containing one optimized contour
 * @return						length of buffer containing one optimized contour (opt_contour)
 */
static uint16_t contour_optimization(edge_track *contour, uint16_t size, edge_track* opt_contour, uint16_t opt_contour_len){


	uint16_t index = 0;
	float dmax = 0;
	float distance = 0;


	for(uint16_t i = 1; i < size; ++i){
		distance = perpendicular_distance(contour[0].pos, contour[size-1].pos, contour[i].pos);
		if(distance > dmax) {
			index = i;
			dmax = distance;
		}
	}
	 /**
	  * If the maximum distance is superior to MAX_PERP_DIST, the contour is divided into 2 subcontours which are
	  * passed recursively into path_optimization again.
	  *	n1 and n2 are the final size of the new subcontours computed recursively and are used to change
	  *	the position of the dest pointer in which we will put the values of both edges of a subcontour
	  */

	if(dmax > MAX_PERP_DIST){
		// Recursive call
		uint16_t n1 = contour_optimization(contour, index + 1, opt_contour, opt_contour_len);
		if (opt_contour_len >= n1 - 1){
			opt_contour_len -= n1 - 1;
			opt_contour += n1 - 1;
		} else {
			opt_contour_len = 0;
		}
		uint16_t n2 = contour_optimization(contour + index, size-index, opt_contour, opt_contour_len);
		return n1 + n2 - 1;

	} else if(dmax == 0) {
		uint16_t k = 0;
		while(k*MAX_PIXEL_DIST < size){
			opt_contour[k] = contour[k*MAX_PIXEL_DIST];
			++k;
		}
		if(!((k-1)*MAX_PIXEL_DIST == size-1)){
			opt_contour[k] = contour[size-1];
			++k;
		}
		return k;
	}

	if(opt_contour_len >= 2) {
		opt_contour[0] = contour[0];
		opt_contour[1] = contour[size - 1];
	}
	return 2;
}


/**
 * @brief							Samples all contours from the contours buffer and optimizes their path one by one
 * @param[in] 	contours			pointer to contour buffer
 * @param[in]	edges				pointer to edges buffer
 * @return		opt_contours_size	length of buffer containing all contours
 */
static uint16_t path_optimization(edge_track* contours, edge_pos* edges, uint16_t size_edges){

	uint16_t opt_contours_size = 0;
	uint16_t contours_size = 0;
	for(uint8_t i = 0; i < (size_edges)/2; ++i){
		uint16_t start_index = edges[i*2].index;
		uint16_t end_index = edges[i*2+1].index;
		uint8_t loop = 0;

		if(start_index == end_index){
			contours[opt_contours_size] = contours[contours_size];
			++opt_contours_size;
			++contours_size;
		} else {
			uint16_t length = end_index - start_index + 1;
			if(contours[start_index].pos.x == contours[end_index].pos.x && contours[start_index].pos.y == contours[end_index].pos.y){
				--length;
				++loop;
			}

			uint16_t k = 0;
			res_out = malloc(length*sizeof(struct edge_track));
			new_contour = malloc(length*sizeof(struct edge_track));
			for(uint16_t n = 0; n < length; ++n){
				new_contour[n] = contours[contours_size + n];
			}

			contours_size += length;
			uint16_t final_size = contour_optimization(new_contour,length, res_out,length);

			for(uint16_t j = opt_contours_size; j < opt_contours_size + final_size; ++j){
				contours[j] = res_out[k];
				++k;
			}

			opt_contours_size += final_size;
			if(loop){
				contours[opt_contours_size] = contours[start_index];
				++opt_contours_size;
				++contours_size;
			}
			free(new_contour);
			free(res_out);
		}
	}
	return opt_contours_size;
}





/**
 * @brief						reorders edges indexes to match optimized contour buffer indexes
 * @param[in] opt_contours_size size (length) of optimized contours buffer
 * @return						none
 */
static void reorder_edges_index(uint16_t opt_contours_size, uint16_t size_edges)
{
	uint16_t k = 0;
//		edges[size_edges - 1].index = total_size - 1;
	uint16_t num_contours = 0;
	for(uint16_t i = 0; i < opt_contours_size; ++i){
		if(contours[i].start_end == 1){
			edges[k].index = i;
			++k;
			++num_contours;
		}
	}
		chprintf((BaseSequentialStream *)&SDU1, "num_contours startend = %d \r\n",num_contours);

	edges[size_edges - 1].index = opt_contours_size - 1;
}


/**
 * @brief						reorders edges buffer to minimize travel distance
 * @return						none
 */
static void nearest_neighbour(uint16_t size_edges){

	uint16_t min_index = 0;
	float min_distance = IM_HEIGHT_PX+IM_LENGTH_PX;
	float distance = 0;

	// first find edge pair closest to initial robot position
	bool first_pos = true;
	cartesian_coord init_pos;
	init_pos.x = INIT_ROBPOS_PX; init_pos.y = INIT_ROBPOS_PY;
	for(uint16_t start_index = 0; start_index < size_edges-1; start_index+=2) {
		min_distance = IM_HEIGHT_PX+IM_LENGTH_PX;
		for(uint16_t i = start_index; i < size_edges; ++i) {
			// search for index with smallest distance
			if (first_pos)
				distance = two_point_distance(edges[i].pos, init_pos);
			else
				distance = two_point_distance(edges[i].pos, edges[start_index-1].pos);
			if(distance < min_distance) {
				min_distance = distance;
				min_index = i;
			}
		}
		struct edge_pos edge_start_temp;
		struct edge_pos edge_end_temp;
		// if min_index is even, smaller index is at min_index
		if (min_index%2 == 0) {
			edge_start_temp = edges[min_index];
			edge_end_temp = edges[min_index+1];
			edges[min_index] = edges[start_index];
			edges[min_index+1] = edges[start_index+1];
			edges[start_index] = edge_start_temp;
			edges[start_index+1] = edge_end_temp;
			status[start_index] = start;

		// if index is odd, smaller index is at min_index-1
		} else {
			edge_start_temp = edges[min_index];
			edge_end_temp = edges[min_index-1];
			edges[min_index-1] = edges[start_index];
			edges[min_index] = edges[start_index+1];
			edges[start_index] = edge_start_temp;
			edges[start_index+1] = edge_end_temp;
			status[start_index] = end;
		}
		// once first index is found, repeat for other edge pairs
		first_pos = false;
	}
}




/**
 * @brief						resizes path buffer to fit into an image of size
 * 								(canvas_size_x) x (canvas_size_y)
 * @param[in]	canvas_size_x	canvas width in pixel
 * @param[in]	canvas_size_y	canvas height in pixel
 * @param[out]	path			pointer to path buffer
 * @return						none
 */
static void img_resize(cartesian_coord* path, uint16_t canvas_size_x, uint16_t canvas_size_y){

	// calculate resize coefficient
	float resize_coeff_x = (float)canvas_size_x/IM_LENGTH_PX;
	float resize_coeff_y = (float)canvas_size_x/IM_HEIGHT_PX;
	float resize_coeff;

	if (resize_coeff_x > resize_coeff_y)
		resize_coeff = resize_coeff_y;
	else
		resize_coeff = resize_coeff_x;

	// resize each position in path buffer
	uint16_t path_length = data_get_length();
	for (uint16_t i = 0; i < path_length; ++i) {
		path[i].x *= resize_coeff;
		path[i].y *= resize_coeff;
	}

}


/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void path_planning(void)
{
	// free previous position buffer
	data_free_pos();

	uint16_t size_edges = 0;
	uint16_t size_contours = 0;

	// get img_buffer containing result from canny edge detection algorithm
	uint8_t* img_buffer = get_img_buffer();
	uint16_t nb_pixels = 0;

	// count number of active pixels (value = IM_MAX_VALUE)
	uint16_t pos = 0;
	for (uint8_t x = 0; x < IM_LENGTH_PX; ++x) {
		for (uint8_t y = 0; y < IM_HEIGHT_PX; ++y) {
			pos = position(x,y);
			if (img_buffer[pos] == IM_MAX_VALUE)
				++nb_pixels;
		}
	}

//	chprintf((BaseSequentialStream *)&SDU1, " nb_pixels %d \r\n", nb_pixels);

	if(nb_pixels == 0)
		return;
	/**
	 * allocate contours and edges with temporary size for edges
	 * in accordance with the line_tracing algorithm, the "worst case scenario" is
	 * when 3 pixels are in an "L" configuration, which would lead to 4 contours
	 * (because the last pixel reattaches to the starting pixel, by design)
	 * Thus, we need to allocate 4 slots per 3 pixels to be safe.
	 */


	contours = calloc(INIT_ARR, sizeof(edge_track));
	edges = calloc(INIT_ARR, sizeof(edge_pos));

	// fill contours and edge buffers and find their correct sizes
	path_tracing(img_buffer, contours, edges, &size_contours, &size_edges);

	// realloc edges and contours to correct size
//	contours = realloc(contours, size_contours*sizeof(edge_track));
//	edges = realloc(edges, size_edges*sizeof(edge_pos));

	// optimize contour by deleting redudant positions in coutours buffer
	uint16_t opt_contours_size = path_optimization(contours,edges, size_edges);

	// reallocate contour with new size
	contours = realloc(contours, opt_contours_size*sizeof(edge_track));

	// reorder edges buffer indexes to match optimized contour
	reorder_edges_index(opt_contours_size, size_edges);

	// reorder the edges to minimize travel distance
	status = calloc(size_edges, sizeof(uint8_t*));
	nearest_neighbour(size_edges);

	// Allocate and fill final_path buffer
	uint16_t total_size = opt_contours_size + 1;
	cartesian_coord* final_path = data_alloc_xy(total_size);

	cartesian_coord init_pos;
	init_pos.x = INIT_ROBPOS_PX; init_pos.y = INIT_ROBPOS_PY;
	final_path[0] = init_pos;

	chprintf((BaseSequentialStream *)&SDU1, " size_edges %d \r\n", size_edges);
	chprintf((BaseSequentialStream *)&SDU1, " size_contours %d \r\n", size_contours);
	chprintf((BaseSequentialStream *)&SDU1, " opt_contours_size %d \r\n", opt_contours_size);
	chprintf((BaseSequentialStream *)&SDU1, " opt_contours_size %d \r\n", total_size);


	// Fill final_path buffer with optimized contour
	uint16_t k = 1;
	for (uint16_t i = 0; i < size_edges; i+=2) {
		if(status[i] == start) {
			for(uint16_t j = edges[i].index; j <= edges[i+1].index; ++j){
				final_path[k].x = contours[j].pos.x;
				final_path[k].y = contours[j].pos.y;
				++k;
			}
		} else if(status[i] == end) {
			for(int16_t j = edges[i].index; j >= edges[i+1].index; --j){ //int16 because j is decremented to -1 for an index of 0
				final_path[k].x = contours[j].pos.x;
				final_path[k].y = contours[j].pos.y;
				++k;
			}
		}
	}


	for (uint16_t i = 0; i < total_size; ++i)
		chprintf((BaseSequentialStream *)&SDU1, " final path %d x:%d y%d \r\n", i, final_path[i].x, final_path[i].y);

	// free buffers
	free(status);
	free(contours);
	free(edges);
}






