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
* contours:
* size: size_contours
* contains positions of each active pixel, ordered so that it follows a contour.
*
* -----------------------------------------------------------------------------
*
* edges:
* size: size_edges
* contains positions of extremities.
* Ordered by pairs, i.e. edges[0] goes with edges[1] and so on.
* note: Index is associated to the pixel position at the
* position <index> in the contours buffer.
*
* -----------------------------------------------------------------------------
*
* final_path:
* size: total_size
* contains the final positions that the robot has to follow.
* note: final_path[0] contains initial robot position
*
* -----------------------------------------------------------------------------
*
* color:
* size: total_size
* contains the color associated to each pixel in final_path.
*
* -----------------------------------------------------------------------------
*
* status:
* size: size_edges
* contains edge status (start, end).
* note: it is separated from the edge_pos structure to avoid padding.
*
* -----------------------------------------------------------------------------
*/

// C standard header files

#include <stdlib.h>
#include <stdint.h>
#include <math.h>

// ChibiOS headers

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

// Module headers

#include <mod_path.h>
#include <tools.h>
#include <mod_img_processing.h>
#include <mod_communication.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define INIT_ROBPOS_PX     50
#define INIT_ROBPOS_PY     0

/** this dictates the maximum perpendicular distance in pixels between
 * approximated lines of optimized contour buffer and corresponding points of
 * the non optimized contour buffer.
 * Smaller values lead to more points for approximating a shape.
 */

#define MAX_PERP_DIST      0.95f

/** max allowed distance between two positions in the final buffer.
 * this parameter is important because the robot needs more than two points
 * to draw a straight line (mechanical constraint)
 */
#define MAX_PIXEL_DIST     3
#define KEEP               1
#define REMOVE             0


/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static edge_pos* edges;
static edge_track* contours;

static uint8_t* contours_to_remove;
static struct edge_track* new_contour;

static uint8_t* status;


/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/


/**
 * @brief                       fills contours and edges buffer
 *                              and determines colors
 * @param[in]   img_buffer      pointer to image buffer
 * @param[in]   color           pointer to color buffer
 * @param[out]  contours        pointer to contour buffer
 * @param[out]  edges           pointer to edges buffer
 * @return                      none
 */
static void path_tracing(uint8_t* img_buffer, uint8_t* color,
		                  edge_track *contours,  edge_pos *edges,
                          uint16_t* size_contours, uint16_t* size_edges)
{
	// max has to be at STRONG_PIXEL (because of output of canny_edge).
	// Other values are chosen arbitrarily.
	enum px_status {max = STRONG_PIXEL, visited = STRONG_PIXEL-1,
	                rewind = STRONG_PIXEL-2, begin = STRONG_PIXEL-3};
	const int8_t dx = 1;
	const int8_t dy = IM_LENGTH_PX;

	bool extremity_found = false;

	uint16_t edge_index = 0;

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

						edges[edge_index].pos.x = x_temp;
						edges[edge_index].pos.y = y_temp;
						edges[edge_index].index = *size_contours;

						contours[*size_contours].pos.x = x_temp;
						contours[*size_contours].pos.y = y_temp;
						contours[*size_contours].is_extremity = true;
						contours[*size_contours].color = color[position(x_temp, y_temp)];

						++edge_index;
					}
					img_buffer[pos] = visited;
				}

				// Once extremity is found, rewind the path and find second extremity.

				// this allows overlapping back on the starting position
				// (e.g. closed curve)
				img_buffer[pos] = begin;

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
					// Note: we don't want to connect back to starting position
					//       if line length is 2 px
					// *size_contours-1 != edges[edge_index-1].index handles this

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
						edges[edge_index].pos.x = x_temp;
						edges[edge_index].pos.y = y_temp;
						// isolated points must be saved as 2 different edges
						if(edges[edge_index-1].index == *size_contours)
							++(*size_contours);
						edges[edge_index].index = *size_contours;
						contours[*size_contours].is_extremity = true;
						++edge_index;
					}
					img_buffer[pos] = rewind;
					if(!extremity_found) {
						++(*size_contours);
						contours[*size_contours].is_extremity = false;
					}
					contours[*size_contours].pos.x = x_temp;
					contours[*size_contours].pos.y = y_temp;
					contours[*size_contours].color = color[position(x_temp, y_temp)];
				}
				++(*size_contours);
			}
		}
	}
	*size_edges = edge_index;
}




/**
 * @brief                       determines the dominant color of a contour and gives
 *                              it to all of its pixels
 * @param[in]   color           pointer to buffer containing path color
 * @param[in]   size_edges      size of the edges buffer
 * @return                      none
 */
static void set_contours_color(uint8_t* color, uint16_t size_edges)
{
		// count number of corresponding color for each edge pair
		uint16_t k = 1, m = 0;
		uint16_t *black_count = calloc(size_edges/2, sizeof(uint16_t*));
		uint16_t *red_count = calloc(size_edges/2, sizeof(uint16_t*));
		uint16_t *green_count = calloc(size_edges/2, sizeof(uint16_t*));
		uint16_t *blue_count = calloc(size_edges/2, sizeof(uint16_t*));
		for (uint16_t i = 0; i < size_edges; i+=2) {
			if(edges[i+1].index > edges[i].index) {
				for(uint16_t j = edges[i].index; j <= edges[i+1].index; ++j){

					switch(color[position(contours[j].pos.x, contours[j].pos.y)]) {
						case black:
							++black_count[m];
							break;
						case red:
							++red_count[m];
							break;
						case green:
							++green_count[m];
							break;
						case blue:
							++blue_count[m];
							break;
					}

					++k;
				}
			} else if(edges[i+1].index < edges[i].index) {
				//int16 because j is decremented to -1 for an index of 0
				for(int16_t j = edges[i].index; j >= edges[i+1].index; --j){

					switch(color[position(contours[j].pos.x, contours[j].pos.y)]) {
						case black:
							++black_count[m];
							break;
						case red:
							++red_count[m];
							break;
						case green:
							++green_count[m];
							break;
						case blue:
							++blue_count[m];
							break;
					}
					++k;
				}
			}
			++m;
		}

		m = 0;
		k = 0;
		for (uint16_t i = 0; i < size_edges; i+=2) {
			uint8_t final_color = white;
			if(red_count[m] > fmax(fmax(green_count[m],blue_count[m]),black_count[m])){
				final_color = red;
			} else if(green_count[m] > fmax(fmax(blue_count[m],red_count[m]),
			                                 black_count[m])){
				final_color = green;
			} else if(blue_count[m] > fmax(fmax(black_count[m],red_count[m]),
			                                green_count[m])){
				final_color = blue;
			}	else final_color = black;

			if(edges[i+1].index > edges[i].index) {
				for(uint16_t j = edges[i].index; j <= edges[i+1].index; ++j){
					contours[k].color = final_color;
					++k;
				}
			} else if(edges[i+1].index < edges[i].index) {
				for(int16_t j = edges[i].index; j >= edges[i+1].index; --j){
					contours[k].color = final_color;
					++k;
				}
			}
			++m;
		}
		free(black_count);
		free(red_count);
		free(green_count);
		free(blue_count);
}

/**
 * @brief                       optimizes the path, i.e. deletes redundant points
 *                              between two edges
 * @param[in]   contour         pointer to buffer containing one contour
 * @param[in]   start           start index of general and subdivised contours
 * @param[in]   end             end index of general and the subdivised contours
 * @param[out]  opt_contour     pointer to buffer containing index of redundant contours
 * @return                      none
 * @details                     if the maximum distance is superior to MAX_PERP_DIST,
 *                              the contour is divided into 2 subcontours for which
 *                              the start and end indexes are saved in their respective
 *                              buffers. If the maximum distance is equal to zero,
 *                              this means that the subcontour is linear and we keep
 *                              one out of MAX_PIXEL_DIST pixel along the line.
 *                              Finally if the maximum distance is inferior to zero,
 *                              we cut all pixels between the start and the end.
 */
static void contour_optimization(edge_track *contour, uint16_t start, uint16_t end,
                                  uint8_t* opt_contour){
	uint16_t start_depth[(end-start)/2];
	uint16_t end_depth[(end-start)/2];

	start_depth[0] = start;
	end_depth[0] = end;

	uint16_t stack_count = 1;

	float distance = 0;

	while (stack_count > 0){

		start = start_depth[stack_count-1];
		end = end_depth[stack_count-1];
		--stack_count;
		start_depth[stack_count] = 0;
		end_depth[stack_count] = 0;

		uint16_t index = start;
		float dmax = 0;

		for(uint16_t i = index+1; i < end; ++i){
			if(opt_contour[i]){
				distance = perpendicular_distance(contour[start].pos, contour[end].pos,
				                                  contour[i].pos);
				if(distance > dmax) {
					index = i;
					dmax = distance;
				}
			}
		}

		if(dmax >= MAX_PERP_DIST){
			start_depth[stack_count] = start;
			end_depth[stack_count] = index;
			++stack_count;
			start_depth[stack_count] = index;
			end_depth[stack_count] = end;
			++stack_count;
		} else if(dmax == 0) {
			uint16_t k = 0;
			for(uint16_t i = start + 1; i < end;++i){
				opt_contour[i] = REMOVE;
			}
			while(k*MAX_PIXEL_DIST + start + 1 < end ){
				opt_contour[start+k*MAX_PIXEL_DIST] = KEEP;
				++k;
				}
			if(!((k-1)*MAX_PIXEL_DIST+start == end)){
				opt_contour[end] = KEEP;
				++k;
				}
		} else {
			for(uint16_t i = start + 1; i < end;++i){
				opt_contour[i] = REMOVE;
			}
		}
	}
}

/**
 * @brief                           samples all contours from the contours buffer and
 *                                  optimizes their path one by one
 * @param[in]   contours            pointer to contour buffer
 * @param[in]   edges               pointer to edges buffer
 * @return      opt_contours_size   length of buffer containing all contours
 */
static uint16_t path_optimization(edge_track* contours, edge_pos* edges,
                                   uint16_t size_edges){

	uint16_t opt_contours_size = 0;
	uint16_t contours_size = 0;
	for(uint8_t i = 0; i < (size_edges)/2; ++i){
		uint16_t start_index = edges[i*2].index;
		uint16_t end_index = edges[i*2+1].index;
		uint8_t loop = 0;

		if(end_index - start_index == 1){
			contours[opt_contours_size] = contours[contours_size];
			contours[opt_contours_size+1] = contours[contours_size+1];
			opt_contours_size += 2;
			contours_size += 2;
		} else {
			uint16_t length = end_index - start_index + 1;
			if(contours[start_index].pos.x == contours[end_index].pos.x
			   && contours[start_index].pos.y == contours[end_index].pos.y){
				--length;
				++loop;
			}

			new_contour = malloc(length*sizeof(struct edge_track));
			contours_to_remove = malloc(length*sizeof(uint8_t));

			for(uint8_t m = 0; m < length; ++m){
				contours_to_remove[m] = KEEP;
			}

			for(uint16_t n = 0; n < length; ++n){
				new_contour[n] = contours[contours_size + n];
			}

			contours_size += length;
			contour_optimization(new_contour, 0, length-1, contours_to_remove);

			for(uint8_t j = 0; j < length; ++j){
				if(contours_to_remove[j] == KEEP){
				contours[opt_contours_size] = new_contour[j];
				++opt_contours_size;
				}
			}

			if(loop){
				contours[opt_contours_size] = contours[contours_size];
				++opt_contours_size;
				++contours_size;
			}
			free(new_contour);
			free(contours_to_remove);
		}
	}
	return opt_contours_size;
}


/**
 * @brief                          reorders edges indexes to match optimized
 *                                 contour buffer indexes
 * @param[in]   opt_contours_size  size (length) of optimized contours buffer
 * @param[in]   size_edgessize     (length) of edges buffer
 * @return                         none
 */
static void reorder_edges_index(uint16_t opt_contours_size, uint16_t size_edges)
{
	uint16_t k = 0;
	for(uint16_t i = 0; i < opt_contours_size; ++i){
		if(contours[i].is_extremity == true){
			edges[k].index = i;
			++k;
		}
	}
	edges[size_edges - 1].index = opt_contours_size - 1;
}


/**
 * @brief                          reorders edges buffer to minimize travel distance
 * @param[in]   size_edges         size (length) of edges buffer
 * @return                         none
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
 * @brief                       fills final_path and color buffers with optimized contour
 *                              and its corresponding colors.
 * @param[in]	size_edges      the size of the edges buffer
 * @param[out] 	color           pointer to buffer containing path color
 * @param[out]	final_path      pointer to buffer containing the coordinates of
 *                              path to be drawn
 * @return                      none
 */
static void create_final_path(uint8_t* color, uint16_t size_edges,
                               cartesian_coord* final_path)
{

	cartesian_coord init_pos;
	init_pos.x = INIT_ROBPOS_PX; init_pos.y = INIT_ROBPOS_PY;
	final_path[0] = init_pos;
	color[0] = white;
	uint16_t k = 1;
	for (uint16_t i = 0; i < size_edges; i+=2) {
		if(edges[i+1].index > edges[i].index) {
			for(uint16_t j = edges[i].index; j <= edges[i+1].index; ++j){
				final_path[k].x = contours[j].pos.x;
				final_path[k].y = contours[j].pos.y;
				if (j == edges[i].index)
					color[k] = white;
				else
					color[k] = contours[j].color;
				++k;
			}
		} else if(edges[i+1].index < edges[i].index) {
			// int16 because j is decremented to -1 for an index of 0
			for(int16_t j = edges[i].index; j >= edges[i+1].index; --j){
				final_path[k].x = contours[j].pos.x;
				final_path[k].y = contours[j].pos.y;
				if (j == edges[i].index)
					color[k] = white;
				else
					color[k] = contours[j].color;
				++k;
			}
		}
	}
}


/**
 * @brief                       resizes path buffer to fit into an image of size
 *                              (canvas_size_x) x (canvas_size_y)
 * @param[in]   canvas_size_x   canvas width in pixel
 * @param[in]   canvas_size_y   canvas height in pixel
 * @param[out]  path            pointer to path buffer
 * @return                      none
 */
static void img_resize(cartesian_coord* path, uint16_t canvas_size_x,
                        uint16_t canvas_size_y){

	// calculate resize coefficient
	float resize_coeff_x = (float)canvas_size_x/IM_LENGTH_PX;
	float resize_coeff_y = (float)canvas_size_y/IM_HEIGHT_PX;
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

	// count number of active pixels (value = STRONG_PIXEL)
	uint16_t pos = 0;
	for (uint8_t x = 0; x < IM_LENGTH_PX; ++x) {
		for (uint8_t y = 0; y < IM_HEIGHT_PX; ++y) {
			pos = position(x,y);
			if (img_buffer[pos] == STRONG_PIXEL)
				++nb_pixels;
		}
	}

	if(nb_pixels == 0)
		return;

	/**
	 * allocate contours and edges with temporary size for edges
	 * in accordance with the line_tracing algorithm, the "worst case scenario" is
	 * when 3 pixels are in an "L" configuration, which would lead to 4 contours
	 * (because the last pixel reattaches to the starting pixel, by design)
	 * Thus, we need to allocate 4 slots per 3 pixels to be safe.
	 */

	contours = calloc(nb_pixels*4/3, sizeof(edge_track));
	edges = calloc(nb_pixels*4/3, sizeof(edge_pos));
	uint8_t* color = data_get_color();

	// fill contours and edge buffers and find their correct sizes
	path_tracing(img_buffer, color, contours, edges, &size_contours, &size_edges);

	// realloc edges and contours to correct size
	contours = realloc(contours, size_contours*sizeof(edge_track));
	edges = realloc(edges, size_edges*sizeof(edge_pos));

	// set color of each contour
	set_contours_color(color, size_edges);

	// optimize contour by deleting redundant positions in contours buffer
	uint16_t opt_contours_size = path_optimization(contours,edges, size_edges);

	// reallocate contour with new size
	contours = realloc(contours, opt_contours_size*sizeof(edge_track));
	// reorder edges buffer indexes to match optimized contour
	reorder_edges_index(opt_contours_size, size_edges);

	// reorder the edges to minimize travel distance
	status = calloc(size_edges, sizeof(uint8_t*));
	nearest_neighbour(size_edges);

	// Allocate and fill final_path and color buffers
	uint16_t total_size = opt_contours_size + 1;
	cartesian_coord* final_path = data_alloc_xy(total_size);
	data_set_length(total_size);
	total_size = data_get_length();

	create_final_path(color, size_edges, final_path);
	data_realloc_color(total_size);

	img_resize(final_path, 200, 200); // magic numbers to define in mod_draw.h

	data_set_ready(true);

	// send path to computer
	com_send_data((BaseSequentialStream *)&SD3, NULL, total_size, MSG_IMAGE_PATH);

	// free buffers
	free(status);
	free(contours);
	free(edges);
}








