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
uint8_t edge_scanning(uint8_t img_buffer, uint8_t count_lab){

	uint8_t counter = 1;
	for(uint8_t x=1; x<IM_LENGTH_PX;++x){
		for(uint8_t y=1; y<IM_HEIGHT_PX;++y){
			if(img_buffer[position(x,y)]){
				if(!img_buffer[position(x+dx[0],y+dy[0])]){
					if(img_buffer[position(x+dx[1],y+dy[1])]){
						label[position(x,y)] = label[position(x+dx[1],y+dy[1])];
						if(img_buffer[position(x+dx[2],y+dy[2])])
							merge(count_lab,label[position(x,y)],label[position(x+dx[2],y+dy[2])]);
					}
					else{
						if(img_buffer[position(x+dx[2],y+dy[2])]){
						label[position(x,y)] = label[position(x+dx[2],y+dy[2])];
						if(img_buffer[position(x+dx[3],y+dy[3])])
							merge(count_lab,label[position(x,y)],label[position(x+dx[3],y+dy[3])]);
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
					label[position(x,y)] = label[position(x+dx[0],y+dy[0])]

				}
			}
		}
	}
	return counter;
}


void path_labelling(uint8_t img_buffer){

	uint8_t counter = 0;
	counter = edge_scanning(img_buffer,count_lab);
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
 * @brief
 */
void path_optimization(void){



}


/**
 * @brief
 */
void nearest_neighbour(void){




}

/**
 * @brief				This function transforms an array of arrays into a single array of 1 dimension elements
 */
uint8_t flatten(uint8_t old_img){

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
uint8_t push_back(uint8_t temp_img_val, uint8_t new_img){

	uint8_t newer_img[sizeof(new_img)/sizeof(uint8_t) + 1] = {0};

	for(uint8_t i = 0; i < sizeof(new_img)/sizeof(uint8_t); ++i)
	newer_img[i] = new_img[i];

	newer_img[sizeof(new_img)/sizeof(uint8_t) + 1] = temp_img_val;
	return newer_img;
}

//
/**
 * @brief		This function gives the same label to 2 pixels with different labels
 */
void merge(uint8_t count_lab, uint8_t x, uint8_t y){

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


