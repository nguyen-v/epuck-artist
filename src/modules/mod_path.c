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

const uint8_t dx[] = {0, -1, +1, -1};
const uint8_t dy[] = {-1, 0, -1, -1};

static uint8_t label[IM_LENGTH_PX*IM_HEIGHT_PX] = {0};
static uint8_t count_lab[IM_LENGTH_PX*IM_HEIGHT_PX] = {0};




/**
 * @brief				Now here's a tricky one. Path scanning is done using a Two pass parallel Connected-component labeling algorithm.
 * 				This algorithm uses a 8 neighbour checking system and raster scanning to attribute labels to defined edges.
 *
 */

uint8_t edge_scanning(uint8_t *img_buffer, uint8_t *count_lab){

	uint8_t counter = 1;
	for(uint8_t x=1; x<IM_LENGTH_PX;++x){
		for(uint8_t y=1; y<IM_HEIGHT_PX;++y){
			if(img_buffer[position(x,y)]){
				if(!img_buffer[position(x+dx[0],y+dy[0])]){
					if(img_buffer[position(x+dx[1],y+dy[1])]){
						label[position(x,y)] = label[position(x+dx[1],y+dy[1])];
						if(img_buffer[position(x+dx[2],y+dy[2])])
							merge(&count_lab,label[position(x,y)],label[position(x+dx[2],y+dy[2])]);
					}
					else{
						if(img_buffer[position(x+dx[2],y+dy[2])]){
						label[position(x,y)] = label[position(x+dx[2],y+dy[2])];
						if(img_buffer[position(x+dx[3],y+dy[3])])
							merge(&count_lab,label[position(x,y)],label[position(x+dx[3],y+dy[3])]);
						}
						else{
							if(img_buffer[position(x+dx[3],y+dy[3])]){
								label[position(x,y)] = label[position(x+dx[3],y+dy[3])];
							}
							else{
								label[position(x,y)] = counter;
								count_lab[counter] = counter;
								++counter;
							}
						}
					}
				}
				else{
					label[position(x,y)] = label[position(x+dx[0],y+dy[0])];

				}
			}
		}
	}
	return counter;
}


edge_track edge_tracing(){

	uint8_t diag_px_priority, temp_x, temp_y, next_x, next_y, last_x, last_y, k, j, used_label_trace = 0;
	uint8_t start_end, tracing_progress, traced_label[100], next_px, end_found, lap_found = 0;
	uint16_t priority = 0;
	px_status status = 0;
	edge_track contours[1000];
	for(uint8_t x=0 ;x < IM_LENGTH_PX;x++){
		for(uint8_t y=0 ; y < IM_HEIGHT_PX;y++){
			if(label[position(x,y)]){
				traced_label[used_label_trace] = label[position(x,y)];
				++used_label_trace;
			}
			uint8_t temp=0;
			while(trace_label[temp] != 0){
				if((label[position(x,y)]) != traced_label[temp]){
					++temp
				}
			}
				if(!tracing_progress){
				temp_x = x;
				temp_y = y;
				tracing_progress = TRUE;
				}
				if(label[position(x,y-1)] == label[position(x,y)]){
					if(x != last_x && y-1 != last_y && next_px == FALSE){
						next_x = x;
						next_y = y-1;
						next_px = TRUE;
					}
					++start_end;
				}
				if(label[position(x-1,y)] == label[position(x,y)]){
					if(x-1 != last_x && y != last_y && next_px == FALSE){
						next_x = x-1;
						next_y = y;
						next_px = TRUE;
					}
					++start_end;
				}
				if((label[position(x+1,y)] == label[position(x,y)]) && start_end < 3){
					if(x+1 != last_x && y != last_y && next_px == FALSE){
						next_x = x+1;
						next_y = y;
						next_px = TRUE;
					}
					++start_end;
				}
				if((label[position(x,y+1)] == label[position(x,y)]) && start_end < 3){
					if(x != last_x && y+1 != last_y && next_px == FALSE){
						next_x = x;
						next_y = y+1;
						next_px = TRUE;
					}
					++start_end;
				}

				if(start_end)
					diag_px_priority = 1;

				if((label[position(x+1,y-1)] == label[position(x,y)]) && start_end < 3){
					if(x != last_x && y+1 != last_y && diag_px_priority && next_px == FALSE){
						next_x = x;
						next_y = y+1;
						next_px = TRUE;
					}
					++start_end;
				}
				if((label[position(x-1,y-1)] == label[position(x,y)]) && start_end < 3){
					if(x != last_x && y+1 != last_y && diag_px_priority && next_px == FALSE){
						next_x = x;
						next_y = y+1;
						next_px = TRUE;
					}
					++start_end;
				}
				if((label[position(x-1,y+1)] == label[position(x,y)]) && start_end < 3){
					if(x != last_x && y+1 != last_y && diag_px_priority && next_px == FALSE){
						next_x = x;
						next_y = y+1;
						next_px = TRUE;
					}
					++start_end;
				}
				if((label[position(x+1,y+1)] == label[position(x,y)]) && start_end < 3){
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

			contours[k].pos_x = x;
			contours[k].pos_y = y;
			contours[k].label = label[position(x,y)];
			contours[k].start_end = status;

			x = next_x;
			y = next_y-1; //goes against y's incrementation
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
 * @brief
 */
void path_planning(void){






}


/**
 * @brief This is an implementation of the Douglas-Pucker algorithm which significantly reduces the total number of points
 * 		  in a contour
 */
void path_optimization(void){







}


/**
 * @brief   tbd...
 */
void nearest_neighbour(void){




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


