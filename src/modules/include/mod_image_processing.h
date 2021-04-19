#ifndef SRC_MODULES_INCLUDE_MOD_IMAGE_PROCESSING_H_
#define SRC_MODULES_INCLUDE_MOD_IMAGE_PROCESSING_H_

enum colour{black, red, blue, green};

void im_acquisition(void);
uint8_t *canny_edge(void);

#endif /* SRC_MODULES_INCLUDE_MOD_IMAGE_PROCESSING_H_ */
