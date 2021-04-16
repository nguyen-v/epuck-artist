/**
 * @file    mod_path.c
 * @brief   Handles the capture and the processing of an image.
 */

// C standard header files

#include <math.h>

// ChibiOS headers

#include <chprintf.h>
#include <usbcfg.h>
#include <include/mod_path.h>

#define IM_LENGTH_PX 100
#define IM_HEIGHT_PX 100
#define INIT_ROBPOS_PX 50
#define INIT_ROBPOS_PY 50

// This value dictates the efficacity of the path optimization algorithm.
// A higher value would give us a path with sharp turns while a lower value would give us curved turns
#define EPSILON 0.02f

const uint8_t dx[] = {0, -1, +1, -1};
const uint8_t dy[] = {-IM_LENGTH_PX, 0, -IM_LENGTH_PX, -IM_LENGTH_PX};

static uint8_t label[IM_LENGTH_PX*IM_HEIGHT_PX] = {0};
static uint8_t count_lab[IM_LENGTH_PX*IM_HEIGHT_PX] = {0};

static uint16_t size_contours = 0;
static uint16_t size_edges = 0;
static uint16_t opt_size_contours = 0;//Those static values should be utilized in order to alloc the necessary memory for the countour and the edge arrays


/**
 * @brief				Now here's a tricky one. Path scanning is done using a Two pass parallel Connected-component labeling algorithm.
 * 				This algorithm uses a 8 neighbour checking system and raster scanning to attribute labels to defined edges.
 *
 */
void edge_scanning(uint8_t *img_buffer, uint8_t *count_lab){

	uint16_t pos = 0;
	uint8_t counter = 1;

	for(uint8_t x=1; x<IM_LENGTH_PX;++x){
		for(uint8_t y=1; y<IM_HEIGHT_PX;++y){
			pos = position(x,y);
			if(img_buffer[pos]){
				if(!img_buffer[pos + dx[0] + dy[0] ]){
					if(img_buffer[pos + dx[1] + dy[1]]){
						label[pos] = label[pos + dx[1] + dy[1]];
						if(img_buffer[pos + dx[2] + dy[2]])
							merge(*count_lab,label[pos],label[pos + dx[2] + dy[2]]);
					}
					else{
						if(img_buffer[pos + dx[2] + dy[2]]){
						label[pos] = label[pos + dx[2] + dy[2]];
						if(img_buffer[pos + dx[3] + dy[3]])
							merge(*count_lab,label[pos],label[pos + dx[3] + dy[3]]);
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
}




void path_labelling(uint8_t *img_buffer){

	edge_scanning(img_buffer,count_lab);
	flatten(*count_lab);
	for(uint8_t x=0 ;x < IM_LENGTH_PX; x++){
		for(uint8_t y=0 ; y < IM_HEIGHT_PX; y++){
			label[position(x,y)]=count_lab[position(x,y)];
		}
	}
}



void edge_tracing(struct edge_track *contours, struct edge_pos *edges){

	uint8_t diag_px_priority, temp_x, temp_y, next_x, next_y, last_x, last_y,j, k = 0;
	uint8_t start_end, tracing_progress, next_px = 0;
	uint8_t l = 1;
	uint16_t pos = 0;
	enum px_status status = 0;
	for(uint8_t x=0 ;x < IM_LENGTH_PX;x++){
		for(uint8_t y=0 ; y < IM_HEIGHT_PX;y++){
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
				save_pos(contours, x, y, label[pos],status,k);
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
 * @brief This is an implementation of the recursive Douglas-Pucker algorithm which significantly reduces the total number of points
 * 		  in a contour.
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
 * @brief   Implementation of the nearest neighbour algorithm. It depends on the position of the start and the end points.
 * 			WHO CARES ABOUT WHAT IT DOES ? IT DOESN'T WORK NO MATTER HOW I IMPLEMENT IT FUCK FUCK FUCK FUCK FUCK FUCK FUCK FUCK...
 */


void nearest_neighbour(struct edge_pos *edges, uint16_t *edge_index){

	uint16_t visited[size_edges] = {0};
	visited[0] = 1;

	uint16_t dmin,d = 0;
	uint16_t algo_finished = 0;
	uint16_t j = 1;

	uint16_t counter_temp = (size_edges - 1) / 2;

	While(!algo_finished){

		uint16_t k = 0;

		uint16_t index = 0;
		struct edge_pos temp = {0};

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
		if(index%2==0){
			//swap(edges[j+1],edges[index]);
			swap(edge_index[j+1],edge_index[index]);
			visited[j+1] = 1;
			//swap(edges[j],edges[index-1]);
			swap(edge_index[j],edge_index[index-1]);
			visited[j] = 1;
			k = index - 1;
		}
		else {
			//swap(edges[j],edges[index]);
			swap(edge_index[j],edge_index[index]);
			visited[j] = 1;
			//swap(edges[j+1],edges[index+1]);
			swap(edge_index[j+1],edge_index[index+1]);
			visited[j] = 1;
			k = index + 1;
		}
		if(counter_temp == size_edges){
			algo_finished = 1;
		}
		j += 2;
	}
}


void swap(struct px_pos *edges1, struct px_pos *edges2){
	uint16_t temp = 0;
	temp = edges1;
	edges1 = edges2;
	edges2 = temp;
}




/**
 * @brief			This functions utilizes all the other functions defined above in order to implement the complete path finding program
 */

struct path_motor path_planning(uint8_t *img_buffer){

	struct edge_track *contours[5000] ={0};			// Unless we find a way to quantify it, the size of this array will be chosen arbitrarily
	struct edge_pos *edges[5000] = {0};				// The same goes for this array
													// Though both can be realocated later on...
	edges[0]->pos.pos_x = INIT_ROBPOS_PX;
	edges[0]->pos.pos_y = INIT_ROBPOS_PY;
	++size_edges;

	//----------- This part labels the contours and extracts them into one single array conveniently named "Contours" ---------------//

	uint16_t result_size = 0;
	path_labelling(img_buffer);
	edge_tracing(contours, edges);
	edges = (struct edge_pos*)realloc(edges, size_edges*sizeof(struct edge_pos));
	contours = (struct edge_track*)realloc(contours, size_contours*sizeof(struct edge_track));  //We realocate the proper memory weight for each array;


	//-------- This part separates all the contours and optimizes them separately before reuniting them together -----------//

	uint16_t total_size = 0;
	uint16_t contours_size = 0;

	for(uint8_t i = 0; i < size_edges/2; ++i){
		uint16_t k = 0;
		uint16_t length = edges[i*2 + 1]->index - edges[i*2]->index + 1;
		struct edge_track res_out[length] = {0};
		struct edge_track *new_contour = (struct edge_track*)malloc(length*sizeof(struct edge_track));
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
		free(new_contour);
	}
	contours = (struct edge_track*)realloc(contours,total_size*sizeof(struct edge_track));


	//-------------------------------- This part decides the final path of the robot -----------------------//

	// We must first find the position of each and every edge again
	uint16_t edge_index[size_edges] = {0};
	uint16_t k = 1;
	edge_index[0] = 0;
	edge_index[size_edges - 1] = total_size - 1;
	for(uint8_t i = 1; i < size_edges-1; ++i){
		if(contours[i]->label != contours[i-1]->label){
			edge_index[k] = i-1;
			edge_index[++k] = i;
			++k;
		}
	}

	//---------------------- This part reorganizes the edges' positions. With this, we know how to read the contours table -----------//
	nearest_neighbor(edges, edge_index);

	for(uint8_t i = 0; i < total_size; ++i){



	}
	return path;
}



void save_pos(struct edge_track *pos, uint8_t x, uint8_t y, uint8_t label, enum px_status start_end, uint8_t k){
	pos[k].pos.pos_x= x;
	pos[k].pos.pos_y = y;
	pos[k].label = label;
	pos[k].start_end = start_end;
}


void save_edge_pos(struct edge_pos *pos, uint8_t x, uint8_t y, uint8_t curve_index, uint8_t this_index){
	pos[this_index].pos.pos_x = x;
	pos[this_index].pos.pos_y = y;
	pos[this_index].index = curve_index;
}







