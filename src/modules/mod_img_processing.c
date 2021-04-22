/**
 * @file    mod_img_processing.c
 * @brief   Handles the capture and the processing of an image.
 */

// C standard header files

#include "include/mod_img_processing.h"

#include <math.h>

// ChibiOS headers

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <camera/po8030.h>
#include <mod_data.h>
#include "camera/dcmi_camera.h"
#include <arm_math.h>


/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

// List of constants used during calculations

#define IM_LENGTH_PX 100
#define IM_HEIGHT_PX 90
#define XY_OFFSET_5x5 2
#define XY_OFFSET_3x3 1
#define RAD2DEG 180./M_PI
#define DEG2RAD M_PI/180.
#define COEFF 0.5f

// Those threshold values were chosen arbitrarily, more values should be tested if possible
// After testing, I noticed that low resolution images have a LOT of noise remaining in the final image.
// The threshold values should be adjusted in order to reduce it to the maximum of our abilities

#define HIGH_THRESHOLD 0.3
#define LOW_THRESHOLD 0.2

// The weights of the gaussian function were directly taken from the internet,
// Would it be wise to test it for different values of the standard deviation ?
// This will mainly affect the blur's effect and thus the final quality of the image...//

#define KER_DIV 159
const uint8_t Gaus5x5[] = { 2,  4,  5,  4, 2,
                         4,  9, 12,  9, 4,
                         5, 12, 15, 12, 5,
                         4,  9, 12,  9, 4,
                         2,  4,  5,  4, 2 };

const int8_t Kx[] = {-1, 0, 1,
				   -2, 0, 2,
				   -1, 0, 1 };

const int8_t Ky[] = {1, 2, 1,
				   0, 0, 0,
				   -1, -2, -1};

static uint8_t *img_buffer;
static enum colour color[IM_LENGTH_PX*IM_HEIGHT_PX];

static float theta[(IM_LENGTH_PX*IM_HEIGHT_PX)]={0};
static float I_mag[(IM_LENGTH_PX*IM_HEIGHT_PX)]={0};

static float max = 0;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief				Captures a single image for a given length and height.
 * 						Those function do not work unless they are put within a thread....
 */
void capture_image(void){
	 /**
	 * @brief   Sets the brigthness of the camera
	 *
	 * @param value         Brightness. [7] = sign (positive if 0) and [6:0] the value. => from -128 to 127
	 *
	 * @return              The operation status.
	 * @retval MSG_OK       if the function succeeded.
	 * @retval MSG_TIMEOUT  if a timeout occurred before operation end
	 * @retval others        see in the implementation for details
	 *
	 */


	 /**
	 * @brief   Sets the contrast of the camera
	 *
	 * @param value         Contrast
	 *
	 * @return              The operation status.
	 * @retval MSG_OK       if the function succeeded.
	 * @retval MSG_TIMEOUT  if a timeout occurred before operation end
	 *
	 */
//	int8_t po8030_set_contrast(uint8_t value);
	po8030_advanced_config(FORMAT_RGB565, 200, 0, 4*IM_LENGTH_PX, 4*IM_HEIGHT_PX, SUBSAMPLING_X4, SUBSAMPLING_X4);
	po8030_set_brightness(64);
	dcmi_disable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	dcmi_capture_start();
	wait_image_ready();
	img_buffer = dcmi_get_last_image_ptr();
	canny_edge();
}

static uint8_t img_temp_buffer[(IM_LENGTH_PX*IM_HEIGHT_PX)] = {0}; // to delete just a test

void send_image(void) {
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer, 4000);
	chThdSleepMilliseconds(400);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+4000, 4000);
	chThdSleepMilliseconds(400);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+8000, 4000);
	chThdSleepMilliseconds(400);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+12000, 4000);
	chThdSleepMilliseconds(400);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+16000, 2000);

}

void send_image_half(void) {
//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer, 4000);
//	chThdSleepMilliseconds(400);
//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+4000, 4000);
//	chThdSleepMilliseconds(400);
//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+8000, 1000);
//	chThdSleepMilliseconds(400);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_temp_buffer, 4000);
	chThdSleepMilliseconds(400);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_temp_buffer+4000, 4000);
	chThdSleepMilliseconds(400);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_temp_buffer+8000, 1000);
	chThdSleepMilliseconds(400);

//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer, 4000);
//	chThdSleepMilliseconds(400);
//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+4000, 4000);
//	chThdSleepMilliseconds(400);
//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+8000, 1000);
//	chThdSleepMilliseconds(400);
}

// IMPORTANT / MAGIC NUMBERS HAVE TO BE DEFINED //

/**
 * @brief				Initalizes all modules.
 */
void canny_edge(void){
//	uint8_t *canny_edge(enum colour *color)

	//image conversion from RGB to grayscale + saves the color for each pixel in a separate array//
//	uint8_t img_temp_buffer[(IM_LENGTH_PX*IM_HEIGHT_PX)] = {0}; // add back
	uint16_t red_px, blue_px, green_px = 0;
	uint16_t low_threshold[3] = {0};
	float average[3] ={0};
	for(uint16_t i = 0; i < (IM_LENGTH_PX * IM_HEIGHT_PX)*2; i+=2){

		red_px = (uint8_t)img_buffer[i] & 0xF8;
		green_px = (uint8_t)((img_buffer[i] & 0x07) || (img_buffer[i+1] & 0xe0));
		blue_px = (uint8_t)img_buffer[i+1] & 0x1F;

		average[0] += red_px;
		average[1] += blue_px;
		average[2] += (uint8_t)green_px/2;
		}

	for(uint8_t i=0; i < 3; ++i)
	low_threshold[i] = (uint16_t)COEFF*average[i];

	for(uint16_t i = 0; i < (IM_LENGTH_PX * IM_HEIGHT_PX)*2; i+=2){

		red_px = (uint8_t)img_buffer[i] & 0xF7;
		green_px = (uint8_t)((img_buffer[i] & 0x07) || (img_buffer[i+1] & 0xe0));
		blue_px = (uint8_t)img_buffer[i+1] & 0x1F;

		if(red_px < low_threshold[0] && blue_px < low_threshold[1] && (uint8_t)green_px/2 < low_threshold[2])
			color[i/2] = black;
		else if(red_px > blue_px && red_px > (uint8_t)green_px/2)
			color[i/2]= red;
		else if(blue_px > (uint8_t)green_px/2 && blue_px > red_px)
			color[i/2] =blue;
		else if((uint8_t) green_px/2 > red_px && (uint8_t) green_px/2 > red_px)
				color[i/2] = green;


		img_buffer[i/2] = (0.2989 * (float)red_px) + (0.5870 * (float)green_px / 2.0) + (0.1140 * (float)blue_px);
	}

	// 5x5 Gaussian Filter. A gaussian function is convoluted to the signal
	// in order to reduce the noise (Kernel convolution was used)//

	uint16_t position = 0;
	for(uint8_t x = XY_OFFSET_5x5; x < IM_LENGTH_PX-XY_OFFSET_5x5; ++x){
		for(uint8_t y = XY_OFFSET_5x5; y < IM_HEIGHT_PX-XY_OFFSET_5x5; ++y){
			position = x + (y * IM_LENGTH_PX);

			uint16_t conv = 0;
			uint16_t k = 0;
			for(int8_t x_ker = -XY_OFFSET_5x5; x_ker <= XY_OFFSET_5x5; ++x_ker){
				for(int8_t y_ker = -XY_OFFSET_5x5; y_ker <= XY_OFFSET_5x5; ++y_ker){
					conv += img_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)]*Gaus5x5[k];
//					__SMMLA(img_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)],Gaus5x5[k],conv);
					++k;
				}
			}
			img_temp_buffer[position] = (uint8_t)(conv / KER_DIV);  //Not sure if this cast is a good idea//
		}
	}

	// Application of the Sobel filters onto the image. //
	// It will gives us the gradient intensity. //

//	float theta[(IM_LENGTH_PX*IM_HEIGHT_PX)]={0};
//	float I_mag[(IM_LENGTH_PX*IM_HEIGHT_PX)]={0};
	max = 0;
	for(uint8_t x = XY_OFFSET_3x3; x < IM_LENGTH_PX-XY_OFFSET_3x3; ++x){
		for(uint8_t y = XY_OFFSET_3x3; y < IM_HEIGHT_PX-XY_OFFSET_3x3; ++y){
			position = x + (y * IM_LENGTH_PX);

			int16_t Ix = 0;
			int16_t Iy = 0;
			uint16_t k = 0;
			for(int8_t x_ker = -XY_OFFSET_3x3; x_ker <= XY_OFFSET_3x3; ++x_ker){
				for(int8_t y_ker = -XY_OFFSET_3x3; y_ker <= XY_OFFSET_3x3; ++y_ker){
					Ix += img_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)]*Kx[k];
					Iy += img_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)]*Ky[k];
//					__SMMLA(img_temp_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)],Kx[k],Ix, img_temp_buffer);
//					__SMMLA(img_temp_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)],Ky[k],Iy);
					++k;
				}
			}
			if(Ix == 0 || Iy == 0){
				I_mag[position] = 0;
			} else {
				I_mag[position] = sqrt(Ix*Ix + Iy*Iy);
				if(I_mag[position]>max)
					max = I_mag[position];
			}

			theta[position] = atan2((float)Ix , (float)Iy);
			}
		}
	chprintf((BaseSequentialStream *)&SDU1, "max %f \r \n",max);
	// local maxima suppression : this process' purpose is to thin the edges by suppressing pixels next to high intensity pixels
	// given their gradient angle

	float i= 0;
	float j= 0;
	for(uint8_t x = XY_OFFSET_3x3; x < IM_LENGTH_PX-XY_OFFSET_3x3; ++x){
		for(uint8_t y = XY_OFFSET_3x3; y < IM_HEIGHT_PX-XY_OFFSET_3x3; ++y){
			position = x + (y * IM_LENGTH_PX);
			if((theta[position] <= 22.5*DEG2RAD && theta[position] >= -22.5*DEG2RAD) || (theta[position] <= -157.5*DEG2RAD && theta[position] >= 157.5*DEG2RAD)){
				i = I_mag[position+1];
				j = I_mag[position-1];
			} else if((theta[position] > 22.5*DEG2RAD && theta[position] <= 67.5*DEG2RAD) || (theta[position] <= -112.5*DEG2RAD && theta[position] > -157.5*DEG2RAD)){
				i = I_mag[position + IM_LENGTH_PX +1];
				j = I_mag[position + IM_LENGTH_PX -1];
			} else if((theta[position] > 67.5*DEG2RAD && theta[position] <= 112.5*DEG2RAD) || (theta[position] <= -67.5*DEG2RAD && theta[position] > -112.5*DEG2RAD)){
				i = I_mag[position + IM_LENGTH_PX ];
				j = I_mag[position - IM_LENGTH_PX ];
			} else if((theta[position] > 112.5*DEG2RAD && theta[position] < 157.5*DEG2RAD) || (theta[position] < -22.5*DEG2RAD && theta[position] >= -67.5*DEG2RAD)){
				i = I_mag[position + IM_LENGTH_PX -1 ];
				j = I_mag[position - IM_LENGTH_PX + 1];
			}
			if(i >= I_mag[position] || j >= I_mag[position])
				img_buffer[position] = 0;
			else
				img_buffer[position] = (uint8_t)I_mag[position];
		}
	}


	// Double threshold - This is used to identify pixel intensities and impose 2 pixel intensities

	for (uint8_t x = 0; x < IM_LENGTH_PX; x++) {
		for (uint8_t y = 0; y < IM_HEIGHT_PX; y++) {
			position = x + (y * IM_LENGTH_PX);
	        if (img_buffer[position] > HIGH_THRESHOLD*max) {
	        	img_temp_buffer[position] = 255;
	        } else if (img_buffer[position] > LOW_THRESHOLD*max) {
	            img_temp_buffer[position] = 100;
	        } else {
	            img_temp_buffer[position] = 0;
	        }
	    }
	}


	// Edge tracking by hysteresis : weak pixels are transformed into strong pixels if and only if one is present around it

//	for (uint8_t x = 0; x < IM_LENGTH_PX; x++) {
//		for (uint8_t y = 0; y < IM_HEIGHT_PX; y++) {
//			position = x + (y * IM_LENGTH_PX);
//			if(img_temp_buffer[position] == 100){
//				if(img_temp_buffer[position-IM_LENGTH_PX-1] == 255|| img_temp_buffer[position-IM_LENGTH_PX] == 255 ||
//						img_temp_buffer[position-IM_LENGTH_PX+1] == 255 || img_temp_buffer[position-1] == 255 ||
//						img_temp_buffer[position+1] == 255 || img_temp_buffer[position+IM_LENGTH_PX-1] == 255 ||
//						img_temp_buffer[position-IM_LENGTH_PX] == 255 || img_temp_buffer[position+IM_LENGTH_PX+1] == 255)
//					img_buffer[position] = 1;
//				else if(img_temp_buffer[position] == 255)
//				img_buffer[position] = 1;
//			else img_buffer[position] = 0;
//			}
//		}
//	//returns a pointer to a binary array
//	}
//return img_buffer;
}


