#ifndef SRC_MODULES_INCLUDE_MOD_IMG_PROCESSING_H_
#define SRC_MODULES_INCLUDE_MOD_IMG_PROCESSING_H_

//enum colour{white, black, red, blue, green}; white = 0, black = 1, red = 2, blue = 3, green = 4
#include "ch.h"
#include "hal.h"


#define IM_LENGTH_PX 100
#define IM_HEIGHT_PX 90

void capture_image(uint8_t *img_buffer, uint8_t* color);
void im_acquisition(void);
void send_image(uint8_t* img_buffer);
void send_image_half(uint8_t* img_buffer);
//uint8_t *canny_edge(enum colour *color);
void canny_edge(uint8_t *img_buffer, uint8_t *color);


#endif /* SRC_MODULES_INCLUDE_MOD_IMG_PROCESSING_H_ */
