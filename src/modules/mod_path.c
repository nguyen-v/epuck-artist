/**
 * @file    mod_path.c
 * @brief   Handles the capture and the processing of an image.
 */

// C standard header files

#include <math.h>

// ChibiOS headers
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <mod_path.h>

#include <mod_img_processing.h>

#define INIT_ROBPOS_PX 50
#define INIT_ROBPOS_PY 0
#define RESIZE_COEF 10

// This value dictates the efficacity of the path optimization algorithm.
// A higher value would give us a path with sharp turns while a lower value would give us curved turns

#define EPSILON 0.02f

const int8_t dx[] = {0, -1, +1, -1};
const int8_t dy[] = {-IM_LENGTH_PX, 0, -IM_LENGTH_PX, -IM_LENGTH_PX};

static uint8_t label[IM_LENGTH_PX*IM_HEIGHT_PX] = {0};

static uint16_t size_contours = 0;
static uint16_t size_edges = 0;

/**
 * @brief			This functions utilizes all the other functions defined above in order to implement the complete path finding program
 */

void path_planning(uint8_t *img_buffer, enum colour *color){

	struct edge_track *contours = (struct edge_track*)malloc(5000*sizeof(struct edge_track));			// Unless we find a way to quantify it, the size of this array will be chosen arbitrarily
	struct cartesian_coord *edges = (struct cartesian_coord*)malloc(5000*sizeof(struct cartesian_coord));			// The same goes for this array
													// Though both can be realocated later on...
	edges[0].x = INIT_ROBPOS_PX;
	edges[0].y = INIT_ROBPOS_PY;
	++size_edges;

	//----------- This part labels the contours and extracts them into one single array conveniently named "Contours" ---------------//

	path_labelling(img_buffer);
	edge_tracing(contours, edges, color);
	edges = (struct cartesian_coord*)realloc(edges, size_edges*sizeof(struct cartesian_coord));
	contours = (struct edge_track*)realloc(contours, size_contours*sizeof(struct edge_track));  //We realocate the proper memory weight for each array;


	//-------- This part separates all the contours and optimizes them separately before reuniting them together -----------//

	uint16_t total_size = 0;
	uint16_t contours_size = 0;

	for(uint8_t i = 0; i < size_edges/2; ++i){
		uint16_t k = 0;
		uint16_t length = edges[i*2 + 1].index - edges[i*2].index + 1;

		struct edge_track res_out[length];
		memset(res_out, 0, length*sizeof(struct edge_track));

		struct edge_track new_contour[length];
		memset(new_contour, 0, length*sizeof(struct edge_track));

		for(uint8_t n = 0; n < length; ++n){
			new_contour[n] = contours[contours_size + n];
		}

		contours_size += length;
		uint16_t final_size = path_optimization(new_contour,length, res_out,length);

		for(uint8_t j = total_size; j < total_size + final_size; ++j){
				contours[j] = res_out[k];
				++k;
		}
		total_size += final_size;
	}
	contours = (struct edge_track*)realloc(contours,total_size*sizeof(struct edge_track));


	//-------------------------------- This part decides the final path of the robot -----------------------//

	// We must first find the position of each and every edge again
	uint16_t edge_index[size_edges];
	memset(edge_index, 0, size_edges*sizeof(uint16_t));

	uint16_t k = 1;
	edge_index[0] = 0;
	edge_index[size_edges - 1] = total_size - 1;
	for(uint8_t i = 1; i < size_edges-1; ++i){
		if(contours[i].label != contours[i-1].label){
			edge_index[k] = i-1;
			edge_index[++k] = i;
			++k;
		}
	}

	//---------------------- This part reorganizes the edges' positions. With this, we know how to read the contours table -----------//


	enum edge_status status[size_edges];
	memset(status, 0, size_edges*sizeof(struct edge_track));

	status[0] = init;
	nearest_neighbour(edges, edge_index, status);
	cartesian_coord* final_path = (cartesian_coord*)malloc(total_size*sizeof(cartesian_coord));
	color = (enum colour*)realloc(final_path,total_size*sizeof(enum colour));


	for(uint8_t i = 0; i < size_edges; i+=2){
		if(status[i] == start){
			for(uint16_t j = edge_index[i]; j <= edge_index[i+1]; ++j){
				final_path[j].x = contours[j].pos.x;
				final_path[j].y = contours[j].pos.y;
				color[j] = contours[j].color;
				if(j == edge_index[i] || j == edge_index[i+1])
					color[j] = white;
			}
		} else {
			if(status[i] == end){
				for(uint16_t j = edge_index[i]; j >= edge_index[i+1]; --j){
					final_path[j].x = contours[j].pos.x;
					final_path[j].y = contours[j].pos.y;
					color[j] = contours[j].color;
					if(j == edge_index[i] || j == edge_index[i+1])
						color[j] = white;
				}
			}
		}
	}
	img_resize(final_path,total_size);
	free(contours);
	free(edges);
}



/**
 * @brief				Now here's a tricky one. Path scanning is done using a Two pass parallel Connected-component labeling algorithm.
 * 				This algorithm uses a 8 neighbour checking system and raster scanning to attribute labels to defined edges.
 *
 */
uint8_t edge_scanning(uint8_t *img_buffer, uint16_t *count_lab){

	uint16_t pos = 0;
	uint8_t counter = 1;

for(uint8_t y=1; y<IM_HEIGHT_PX;++y){
	for(uint8_t x=1; x<IM_LENGTH_PX-1;++x){
			pos = position(x,y);
			if(img_buffer[pos]){
				if(!img_buffer[pos + dx[0] + dy[0] ]){
					if(img_buffer[pos + dx[1] + dy[1]]){

						label[pos] = label[pos + dx[1] + dy[1]];
						if(img_buffer[pos + dx[2] + dy[2]]){
							merge(count_lab,label[pos],label[pos + dx[2] + dy[2]]);
						}
					}
					else{
						if(img_buffer[pos + dx[2] + dy[2]]){
							label[pos] = label[pos + dx[2] + dy[2]];
							if(img_buffer[pos + dx[3] + dy[3]]){
								merge(count_lab,label[pos],label[pos + dx[3] + dy[3]]);
							}
						}
						else{
							if(img_buffer[pos + dx[3] + dy[3]]){
								label[pos] = label[pos + dx[3] + dy[3]];
							}
							else{
								label[pos] = counter;
								count_lab[counter] = counter;
								++counter;
							}
						}
					}
				}
				else{
					label[pos] = label[pos + dx[0] + dy[0]];
				}
			}
		}
	}
	//counter est censé être le label max trouvé
	return counter;
}




void path_labelling(uint8_t *img_buffer){

	uint16_t count_lab[IM_LENGTH_PX] = {0};
	uint8_t counter = edge_scanning(img_buffer,count_lab);
	flatten(count_lab,counter);
	for(uint8_t x=0 ;x < IM_LENGTH_PX; x++){
		for(uint8_t y=0 ; y < IM_HEIGHT_PX; y++){
			label[position(x,y)]=count_lab[label[position(x,y)]];
		}
	}
}


/**
 * @brief This algorithm defines and isolates all the contours from the background. when a contour point is found, it checks for all the neighbours in a 8 pixel
 * 		  area around it and follows the countour until it reaches the edge. With this algorithm, we can tell apart edges from simple lines depending on the number
 * 		  of neighbours around them.
 */
void edge_tracing(struct edge_track *contours, struct cartesian_coord *edges, enum colour *color){

	uint8_t diag_px_priority = 0, temp_x = 0, temp_y = 0, next_x = 0, next_y = 0, last_x = 0, last_y = 0, j = 0, k = 0;
	uint8_t start_end = 0, tracing_progress = 0, next_px = 0;
	uint8_t l = 1;
	uint16_t pos = 0;
	enum px_status status = 0;
	for(uint8_t x = 0 ; x < IM_LENGTH_PX; x++){
		for(uint8_t y = 0 ; y < IM_HEIGHT_PX; y++){
			//if(label[position(x,y)]){
			pos = position(x,y);
			while(label[pos]){
				if(!tracing_progress){
				temp_x = x;
				temp_y = y;
				tracing_progress = TRUE;
				}
				if(label[pos - IM_LENGTH_PX] == label[pos]){
					if(x != last_x && y-1 != last_y && next_px == FALSE){
						next_x = x;
						next_y = y-1;
						next_px = TRUE;
					}
					++start_end;
				}
				if(label[pos - 1] == label[pos]){
					if(x-1 != last_x && y != last_y && next_px == FALSE){
						next_x = x-1;
						next_y = y;
						next_px = TRUE;
					}
					++start_end;
				}
				if((label[pos + 1] == label[pos]) && start_end < 3){
					if(x+1 != last_x && y != last_y && next_px == FALSE){
						next_x = x+1;
						next_y = y;
						next_px = TRUE;
					}
					++start_end;
				}
				if((label[pos + IM_LENGTH_PX] == label[pos]) && start_end < 3){
					if(x != last_x && y+1 != last_y && next_px == FALSE){
						next_x = x;
						next_y = y+1;
						next_px = TRUE;
					}
					++start_end;
				}

				if(start_end)
					diag_px_priority = 1;

				if((label[pos + 1 + IM_LENGTH_PX] == label[pos]) && start_end < 3){
					if(x != last_x && y+1 != last_y && diag_px_priority && next_px == FALSE){
						next_x = x;
						next_y = y+1;
						next_px = TRUE;
					}
					++start_end;
				}
				if((label[pos - 1 - IM_LENGTH_PX] == label[pos]) && start_end < 3){
					if(x != last_x && y+1 != last_y && diag_px_priority && next_px == FALSE){
						next_x = x;
						next_y = y+1;
						next_px = TRUE;
					}
					++start_end;
				}
				if((label[pos - 1 + IM_LENGTH_PX] == label[pos]) && start_end < 3){
					if(x != last_x && y+1 != last_y && diag_px_priority && next_px == FALSE){
						next_x = x;
						next_y = y+1;
						next_px = TRUE;
					}
					++start_end;
				}
				if((label[pos + 1 + IM_LENGTH_PX] == label[pos]) && start_end < 3){
					if(x != last_x && y+1 != last_y && diag_px_priority && next_px == FALSE){
						next_x = x;
						next_y = y+1;
						next_px = TRUE;
					}
					++start_end;
				}
//				start_end += (label[position(x+1,y-1)] == label[position(x,y)]) && start_end < 3 ? 1 : 0;
//				start_end += (label[position(x-1,y-1)] == label[position(x,y)]) && start_end < 3 ? 1 : 0;
//				start_end += (label[position(x-1,y+1)] == label[position(x,y)]) && start_end < 3 ? 1 : 0;
//				start_end += (label[position(x+1,y+1)] == label[position(x,y)]) && start_end < 3 ? 1 : 0;

				if(start_end){
					if(start_end == 1){
						status = edge;
						save_edge_pos(edges, x, y, k, l);
						++l;
						++j;
						++size_edges;
					} else{
						if(start_end == 2) status = line;
					}
					++size_contours;
				}
				save_pos(contours, x, y, label[pos],status, color[pos], k);
				++k;

				label[position(x,y)] = 0; // MAGIC VALUE -> Transforms label into background
				x = next_x;
				y = next_y;
				pos = position(x,y);
				next_px = FALSE;
				start_end = 0;

				if(j == 2){
					x = temp_x;
					y = temp_y;
					j = 0;
				}
			}
		}
	}
}

/**
 * @brief This is an implementation of a path optimization algorithm which significantly reduces the total number of points
 * 		  in a contour. This is accomplished by measuring the maximum perpendicular distance between two point boundaries and all the other points
 * 		  in a chosen countour, then remeasure the max perpendicular distance between two obtained curves given by the [start;index] points and the [index;end] points.
 * 		  The EPSILON threshold can be varied and is used in order to choose which points will be cut. The higher the value, the sharpest our contour will be.
 */
uint16_t path_optimization(struct edge_track *contours, uint16_t size, struct edge_track* dest, uint16_t destlen){

	uint16_t index = 0;
	float dmax = 0;
	float distance = 0;

	for(uint16_t i = 1; i + 1 < size  ; ++i)
	{
		distance = perpendicular_distance(contours[0].pos, contours[size-1].pos, contours[i].pos);
		if(distance > dmax) {
			index = i;
			dmax = distance;
		}
	}

	if(dmax > EPSILON){
		// Appel récursif
		uint16_t n1 = path_optimization(contours, index + 1, dest, destlen);
		if (destlen >= n1 - 1){
			destlen -= n1 - 1;
			dest += n1 - 1;
		} else {
			destlen = 0;
		}
		uint16_t n2 = path_optimization(contours + index, size-index, dest, destlen);
		return n1 + n2 - 1;
	}
	if(destlen >= 2) {
		dest[0] = contours[0];
		dest[1] = contours[size - 1];
	}
	return 2;
}


/**
 * @brief   Implementation of the nearest neighbour algorithm. It calculates the shortest distance between its actual point and all other start/end points
 * 			If it finds one, the algorithm switches their index position in the edge_index array, notes both edges of the contour as being visited
 * 			and recalculates the distance from the newly found point to all the other unvisited points.
 *
 */
void nearest_neighbour(struct cartesian_coord *edges, uint16_t *edge_index, enum edge_status *status){

	uint16_t visited[size_edges]; //remplacer par size_edges
	memset(visited, 0, size_edges*sizeof(struct edge_track));

	visited[0] = 1;


	uint8_t algo_finished = 0;
	uint16_t j = 1;

	uint16_t counter = 0;
	uint16_t counter_max = (size_edges - 1) / 2;
	uint16_t k = 0;

	do{
	float dmin = 0,d = 0;
	uint16_t index = 0;

	for(uint16_t i = 0; i < size_edges; ++i){

		if(!visited[i]){
			d = two_point_distance(edges[k], edges[i]);
			if(dmin == 0)
				dmin = d;
			if(d <= dmin){
				dmin = d;
				index = i;
			}
		}
	}
	if(index == 1){
	break;
	}
	if(index == 2){
		uint16_t temp = 0;
		struct cartesian_coord px_temp = {0};
		temp = edge_index[j];
		px_temp = edges[j];
		edge_index[j] = edge_index[index];
		edges[j] = edges[index];
		edge_index[index] = temp;
		edges[index] = px_temp;
		status[j] = end;
		status[j+1] = start;
		visited[j] = 1;
		visited[j+1] = 1;
		break;
	}

	if(index%2==0){
		//swap(edges[j+1],edges[index]);
		uint16_t temp = 0;
		struct cartesian_coord px_temp = {0};
		temp = edge_index[j];
		px_temp = edges[j];
		edge_index[j] = edge_index[index];
		edges[j] = edges[index];
		edge_index[index] = temp;
		edges[index] = px_temp;
		status[j] = end;
		visited[j] = 1;
		//swap(edges[j],edges[index-1]);
		temp = edge_index[j+1];
		px_temp = edges[j+1];
		edge_index[j+1] = edge_index[index-1];
		edges[j+1] = edges[index-1];
		edge_index[index-1] = temp;
		edges[index-1] = px_temp;
		status[j+1] = start;
		visited[j+1] = 1;
		k = j+1;
		++counter;
	}
	else {
		//swap(edges[j],edges[index]);
		uint16_t temp = 0;
		struct cartesian_coord px_temp = {0};
		temp = edge_index[j];
		px_temp = edges[j];
		edge_index[j] = edge_index[index];
		edges[j] = edges[index];
		edge_index[index] = temp;
		edges[index] = px_temp;
		status[j] = start;
		visited[j] = 1;
		//swap(edges[j+1],edges[index+1]);
		temp = edge_index[j+1];
		px_temp = edges[j+1];
		edge_index[j+1] = edge_index[index+1];
		edges[j+1] = edges[index+1];
		edge_index[index+1] = temp;
		edges[index+1] = px_temp;
		status[j+1] = end;
		visited[j + 1] = 1;
		k = j + 1;
		++counter;
	}
	if(counter_max == counter){
		algo_finished = 1;
	}
	j += 2;
	}
	while(!algo_finished);
}


void swap(uint16_t *edges1, uint16_t *edges2){
	uint16_t temp = 0;
	temp = *edges1;
	edges1 = edges2;
	*edges2 = temp;
}


void img_resize(struct cartesian_coord* path, uint16_t path_size){

	for(uint16_t j = 0; j < path_size; ++j){
		path[j].x *= RESIZE_COEF;
		path[j].y *= RESIZE_COEF;
	}
}


void save_pos(struct edge_track *pos, uint8_t x, uint8_t y, uint8_t label, enum px_status start_end, enum colour color, uint8_t k){
	pos[k].pos.x= x;
	pos[k].pos.y = y;
	pos[k].label = label;
	pos[k].color = color;
	pos[k].start_end = start_end;
}


void save_edge_pos(struct cartesian_coord *pos, uint8_t x, uint8_t y, uint8_t curve_index, uint8_t this_index){
	pos[this_index].x = x;
	pos[this_index].y = y;
	pos[this_index].index = curve_index;
}


uint16_t position(uint8_t pos_x, uint8_t pos_y){

	uint16_t position = pos_x + pos_y*IM_LENGTH_PX;
	return position;

}

void flatten(uint16_t *count_lab, uint16_t count){

	uint16_t k = 1;
	for(uint16_t i=1 ;i<=count; i++){
		if(count_lab[i] < i){
			count_lab[i]=count_lab[count_lab[i]];
		}
		else{
			count_lab[i]=k;
			k++;
		}
	}
}


/**
 * @brief		This function gives the same label to 2 pixels with different labels
 */
uint16_t merge(uint16_t *count_lab, uint16_t x, uint16_t y){

	uint16_t rootx=x, rooty=y, z;
	while(count_lab[rootx]!=count_lab[rooty]){
		if(count_lab[rootx]>count_lab[rooty]){
			if(count_lab[rootx]==rootx){
				count_lab[rootx]=count_lab[rooty];
				return count_lab[rootx];
			}
			z=count_lab[rootx];
			count_lab[rootx]=count_lab[rooty];
			rootx=z;
		}
		else{
			if(rooty==count_lab[rooty]){
				count_lab[rooty]=count_lab[rootx];
				return count_lab[rootx];
			}
			z=count_lab[rooty];
			count_lab[rooty]=count_lab[rootx];
			rooty=z;
		}
	}
	return (count_lab[rootx]);
}




