/**
 * @file    mod_path.c
 * @brief   Handles the capture and the processing of an image.
 */

// C standard header files

#include <math.h>

// ChibiOS headers

#include <chprintf.h>
#include <usbcfg.h>
#include <camera/po8030.h>
#include <mod_communication.h>
#include <include/image_processing.h>
#include <include/mod_path.h>

#define IM_LENGTH_PX 100
#define IM_HEIGHT_PX 90

// This value dictates the efficacity of the path optimization algorithm.
// A higher value would give us a path with sharp turns while a lower value would give us curved turns
#define EPSILON 0.02f

const uint8_t dx[] = {0, -1, +1, -1};
const uint8_t dy[] = {-IM_LENGTH_PX, 0, -IM_LENGTH_PX, -IM_LENGTH_PX};

static uint8_t label[IM_LENGTH_PX*IM_HEIGHT_PX] = {0};
static uint8_t count_lab[IM_LENGTH_PX*IM_HEIGHT_PX] = {0};




/**
 * @brief				Now here's a tricky one. Path scanning is done using a Two pass parallel Connected-component labeling algorithm.
 * 				This algorithm uses a 8 neighbour checking system and raster scanning to attribute labels to defined edges.
 *
 */

uint8_t edge_scanning(uint8_t *img_buffer, uint8_t *count_lab){

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
							merge(&count_lab,label[pos],label[pos + dx[2] + dy[2]]);
					}
					else{
						if(img_buffer[pos + dx[2] + dy[2]]){
						label[pos] = label[pos + dx[2] + dy[2]];
						if(img_buffer[pos + dx[3] + dy[3]])
							merge(&count_lab,label[pos],label[pos + dx[3] + dy[3]]);
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
	return counter;
}


struct edge_track edge_tracing(){

	uint8_t diag_px_priority, temp_x, temp_y, next_x, next_y, last_x, last_y, k, j, used_label_trace = 0;
	uint8_t start_end, tracing_progress, traced_label[100], next_px, end_found, lap_found = 0;
	uint16_t priority, pos = 0;
	enum px_status status = 0;
	struct edge_track contours[1000] ={0};
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
					if(start_end == 1)
						status = edge;
					else if(start_end == 2) status = line;
					else break; // A chprintf should be added here in case something abnormal happens with the image processing algorithm, or the labeling process
				}

				save_pos(*contours, x, y, label[pos],status,k);
				label[position(x,y)] = 0; // MAGIC VALUE -> Transforms label into background
				x = next_x;
				y = next_y;
				pos = position(x,y);
				++k;
				next_px = FALSE;
				if(status == edge){
					++j;
					if(j == 2){
						x = temp_x;
						y = temp_y;
						j = 0;
					}
				}
			}
		}
	}
}

void path_labelling(uint8_t *img_buffer){

	uint8_t counter = 0;
	counter = edge_scanning(&img_buffer,count_lab);
	flatten(count_lab,counter);
	for(uint8_t x=0 ;x < IM_LENGTH_PX;x++){
		for(uint8_t y=0 ; j < IM_HEIGHT_PX;j++){
			label[position(x,y)]=p[position(x,y)];
		}
	}
}

/**
 * @brief This is an implementation of the recursive Douglas-Pucker algorithm which significantly reduces the total number of points
 * 		  in a contour.
 */
uint16_t path_optimization(struct edge_track *contours,  uint16_t start, uint16_t end){

	float dmax = 0;
	float distance = 0;
	uint16_t index = 0;
	for(uint16_t i = start; i <= end  ; ++i)
	{

		distance = perpendicular_distance(contours[start], contours[end], contours[i]);
		if(distance > dmax) {
			index = i;
			dmax = distance;
		}
	}

	uint16_t full_size = (end-start);
	uint16_t *resultList = malloc((full_size)*sizeof(uint16_t));
	uint16_t res1_size = (index - start);
	uint16_t *recResults1 = malloc(res1_size*sizeof(uint16_t));
	uint16_t res2_size = (end-index);
	uint16_t *recResults2 = malloc(res2_size*sizeof(uint16_t));

	// Do not forget to find a way to free them in the main path_finding function !!!! //

	if(dmax > EPSILON){
		// Appel récursif
		recResults1 = path_optimization(&contours, start, index);
		recResults2 = path_optimization(&contours, index, end);
		resultList = array_concatenation(resultList,recResults1,recResults2,res1_size,res2_size);
	}
	else {
		resultList = array_concatenation(resultList,contours[0],contours[end],1,1);
	}
	return resultList;
}


/**
 * @brief   Implementation of the nearest neighbour algorithm. It depends on the position of the start and the end points.
 */
void nearest_neighbour(void){






}


/**
 * @brief
 */
void path_planning(void){






}




/**
 * @brief				This function transforms an array of arrays into a single array of 1 dimension elements
 */
uint8_t flatten(uint8_t *old_img){

	uint8_t temp_img[] = {0};
	if(sizeof(old_img) == 1)
		return old_img;
	uint8_t new_img[] = {0};
	for(uint8_t i=0; i<=sizeof(old_img); i++){
			temp_img = flatten(old_img(i));
			for(uint8_t j = 0; j<= sizeof(temp_img); ++j){
				new_img = push_back(temp_img[j], new_img);
			}
		}
	return new_img;
}

// There is no way in hell I'm playing with all of that malloc/realloc/free bullshit
/**
 * @brief				A rewriting of the C++ function "push_back()"
 */
uint8_t push_back(uint8_t temp_img_val, uint8_t *new_img){

	uint8_t newer_img[sizeof(new_img)/sizeof(uint8_t) + 1] = {0};

	for(uint8_t i = 0; i < sizeof(new_img)/sizeof(uint8_t); ++i)
	newer_img[i] = new_img[i];

	newer_img[sizeof(new_img)/sizeof(uint8_t) + 1] = temp_img_val;
	return newer_img;
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

void save_pos(struct edge_track *pos, uint8_t x, uint8_t y, uint8_t label,px_status start_end, uint8_t k){
	pos[k]->pos->pos_x= x;
	pos[k]->pos->pos_y = y;
	pos[k]->label = label;
	pos[k]->start_end = start_end;
}

uint16_t perpendicular_distance(struct px_pos start, struct px_pos end, struct px_pos point){

	uint8_t line_x = end.pos_x - start.pos_x;
	uint8_t line_y = end.pos_y - start.pos_y;

	uint8_t vec_x = point.pos_x - start.pos_x;
	uint8_t vec_y = point.pos_y - start.pos_y;

	uint8_t vec_x2 = point.pos_x - end.pos_x;
	uint8_t vec_y2 = point.pos_x - end.pos_y;

	uint16_t dot_prod1 = (line_x * vec_x + line_y * vec_y);
	uint16_t dot_prod2 = (line_x * vec_x2 + line_y * vec_y2);

	uint16_t distance = 0;

	if (dot_prod1 > 0)
		distance = arm_sqrt_q15(vec_x2*vec_x2 + vec_y2*vec_y2);
	else if(dot_prod2 < 0)
		distance =  arm_sqrt_q15(vec_x*vec_x + vec_y*vec_y);
	else {
		uint16_t mod = arm_sqrt_q15(line_x*line_x + line_y*line_y);
		distance = abs(line_x*vec_y - line_y*vec_x) / mod;
	}
return distance;
}



void *array_concatenation(uint16_t *result, uint16_t *res1, uint16_t *res2, uint16_t size1, uint16_t size2){

	    memcpy(result, res1, size1*sizeof(uint16_t));
	    memcpy(result + size1, res2, size2*sizeof(uint16_t));
	    return result;


}



