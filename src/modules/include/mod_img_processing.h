#ifndef SRC_MODULES_INCLUDE_MOD_IMG_PROCESSING_H_
#define SRC_MODULES_INCLUDE_MOD_IMG_PROCESSING_H_

enum colour{white, black, red, blue, green};

#define IM_LENGTH_PX 100
#define IM_HEIGHT_PX 90

void capture_image(void);
void im_acquisition(void);
void send_image(void);
void send_image_half(void);
//uint8_t *canny_edge(enum colour *color);
void canny_edge(void);


#endif /* SRC_MODULES_INCLUDE_MOD_IMG_PROCESSING_H_ */
