/*
 * @file	mod_img_processing.h
 * @brief	Exported functions and constants related to
 * 			image processing and capture
 */

#ifndef _MOD_IMG_PROCESSING_H_
#define _MOD_IMG_PROCESSING_H_

/*===========================================================================*/
/* Module exported constants.                                                */
/*===========================================================================*/

#define IM_LENGTH_PX       100
#define IM_HEIGHT_PX       90
#define STRONG_PIXEL       255

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef struct rgb_color {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} rgb_color;

enum Octants{first_octant = 1, second_octant, third_octant, fourth_octant,
             fifth_octant, sixth_octant, seventh_octant, eighth_octant
};

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/**
 * @brief                       creates image capture and image processing threads
 * @return                      none
 */
void mod_img_processing_init(void);

/**
 * @brief                       captures an image from camera and calls
 *                              image processing and path tracing functions
 * @return                      none
 */
void capture_image(void);

/**
 * @brief                       returns image_buffer
 * @return                      pointer to image_buffer
 */
uint8_t* get_img_buffer(void);

// THESE WILL BE PROPERLY DEFINED IN mod_communication (see mod_state branch)
void send_image(void);
void send_image_half(void);

#endif /* _MOD_IMG_PROCESSING_H_ */
