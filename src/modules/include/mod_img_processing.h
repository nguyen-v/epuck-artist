/*
 * @file	mod_path.h
 * @brief	Exported functions and constants related to
 * 			image processing and capture
 */

#ifndef _MOD_IMG_PROCESSING_H_
#define _MOD_IMG_PROCESSING_H_

/*===========================================================================*/
/* Module exported constants.                                                */
/*===========================================================================*/

#define IM_LENGTH_PX 100
#define IM_HEIGHT_PX 90
#define IM_MAX_VALUE 255

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef struct hsl_color {
	float hue;	// hue
	float sat;	// saturation
	float lum;	// luma
} hsl_color;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/**
 * @brief						captures an image from camera and calls
 * 								image processing and path tracing functions
 * @return						none
 */
void capture_image(void);

/**
 * @brief						returns image_buffer
 * @return						pointer to image_buffer
 */
uint8_t* get_img_buffer(void);

// rewrite these
void send_image(void);
void send_image_half(void);

#endif /* _MOD_IMG_PROCESSING_H_ */
