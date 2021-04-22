/**
 * @file    mod_img_processing.c
 * @brief   Handles the capture and the processing of an image.
 */

// C standard header files

#include <math.h>

// ChibiOS headers

#include <chprintf.h>
#include <usbcfg.h>
#include <camera/po8030.h>
#include <include/mod_data.h>
#include "camera/dcmi_camera.h"
#include <arm_math.h>
#include <modules/include/mod_image_processing.h>
#include "ch.h"
#include "hal.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

// List of constants used during calculations

#define IM_LENGTH_PX 100
#define IM_HEIGHT_PX 90
#define XY_OFFSET_5x5 2
#define XY_OFFSET_3x3 1
#define RAD2DEG 180/M_PI
#define COEFF 0.5f

// Those threshold values were chosen arbitrarily, more values should be tested if possible
// After testing, I noticed that low resolution images have a LOT of noise remaining in the final image.
// The threshold values should be adjusted in order to reduce it to the maximum of our abilities

#define HIGH_THRESHOLD 200
#define LOW_THRESHOLD 100

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


/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief				Captures a single image for a given length and height.
 * 						Those function do not work unless they are put within a thread....
 */
static THD_WORKING_AREA(waCaptureProcessImage, 1024);
static THD_FUNCTION(CaptureProcessImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	po8030_advanced_config(FORMAT_RGB565, 0, 10, IM_LENGTH_PX, IM_HEIGHT_PX, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	//dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	while(1){

		dcmi_capture_start();
		wait_image_ready();
		if(image_is_ready()){

		}


	}
}

void img_processing_start(void){
	chThdCreateStatic(waCaptureProcessImage, sizeof(waCaptureProcessImage), NORMALPRIO, CaptureProcessImage, NULL);
}

// IMPORTANT / MAGIC NUMBERS HAVE TO BE DEFINED //

/**
 * @brief				Initalizes all modules.
 */
uint8_t *canny_edge(uint8_t *img_buffer, enum colour *color){


	//image conversion from RGB to grayscale + saves the color for each pixel in a separate array//
	img_buffer = dcmi_get_last_image_ptr();
	uint8_t img_temp_buffer[(IM_LENGTH_PX*IM_HEIGHT_PX)] = {0};
	uint16_t red_px, blue_px, green_px = 0;
	uint16_t low_threshold[3] = {0};
	float average[3] ={0};
	for(uint16_t i = 0; i < IM_LENGTH_PX * IM_HEIGHT_PX; i+=2){

		red_px = (uint8_t)img_buffer[i/2] & 0xF8;
		green_px = (uint8_t)((img_buffer[i/2] & 0x07) || (img_buffer[i/2+1] & 0xe0));
		blue_px = (uint8_t)img_buffer[i/2+1] & 0x1F;

		average[0] += red_px;
		average[1] += blue_px;
		average[2] += (uint8_t)green_px/2;
		}

	for(uint8_t i=0; i < 3; ++i)
	low_threshold[i] = (uint16_t)COEFF*average[i];

	for(uint16_t i = 0; i < IM_LENGTH_PX * IM_HEIGHT_PX; i+=2){

		red_px = (uint8_t)img_buffer[i/2] & 0xF8;
		green_px = (uint8_t)((img_buffer[i/2] & 0x07) || (img_buffer[i/2+1] & 0xe0));
		blue_px = (uint8_t)img_buffer[i/2+1] & 0x1F;

		if(red_px < low_threshold[0] && blue_px < low_threshold[1] && (uint8_t)green_px/2 < low_threshold[2])
			color[i/2] = black;
		else if(red_px > blue_px && red_px > (uint8_t)green_px/2)
			color[i/2]= red;
		else if(blue_px > (uint8_t)green_px/2 && blue_px > red_px)
			color[i/2] =blue;
		else if((uint8_t) green_px/2 > red_px && (uint8_t) green_px/2 > red_px)
				color[i/2] = green;

		img_buffer[i/2] = (0.2126 * red_px) + (0.7152 * green_px / 2.0) + (0.0722 * blue_px);
	}

	// 5x5 Gaussian Filter. A gaussian function is convoluted to the signal
	// in order to reduce the noise (Kernel convolution was used)//

	uint16_t position = 0;
	for(uint8_t x = XY_OFFSET_5x5; x < IM_LENGTH_PX-XY_OFFSET_5x5; ++x){
		for(uint8_t y = XY_OFFSET_5x5; y < IM_HEIGHT_PX-XY_OFFSET_5x5; ++y){
			position = x + (y * IM_LENGTH_PX);

			uint16_t conv = 0;
			uint16_t k = 0;
			for(uint8_t x_ker = -XY_OFFSET_5x5; x_ker > XY_OFFSET_5x5; ++x_ker){
				for(uint8_t y_ker = -XY_OFFSET_5x5; y_ker > XY_OFFSET_5x5; ++y_ker){
//					conv += img_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)]*Gaus5x5[k];
					__SMMLA(img_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)],Gaus5x5[k],conv);
					++k;
				}
			}
			img_temp_buffer[position] = (uint8_t)(conv / KER_DIV);  //Not sure if this cast is a good idea//
		}
	}
	// Application of the Sobel filters onto the image. //
	// It will gives us the gradient intensity. //

	float theta[(IM_LENGTH_PX*IM_HEIGHT_PX)/2]={0};
	float I_mag[(IM_LENGTH_PX*IM_HEIGHT_PX)/2]={0};

	for(uint8_t x = XY_OFFSET_3x3; x < IM_LENGTH_PX-XY_OFFSET_3x3; ++x){
		for(uint8_t y = XY_OFFSET_3x3; y < IM_HEIGHT_PX-XY_OFFSET_3x3; ++y){
			position = x + (y * IM_LENGTH_PX);

			uint16_t Ix, Iy, k = 0;
			for(uint8_t x_ker = -XY_OFFSET_3x3; x_ker > XY_OFFSET_3x3; ++x_ker){
				for(uint8_t y_ker = -XY_OFFSET_3x3; y_ker > XY_OFFSET_3x3; ++y_ker){
//					Ix += img_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)]*Kx[k];
//					Iy += img_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)]*Ky[k];
					__SMMLA(img_temp_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)],Kx[k],Ix);
					__SMMLA(img_temp_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)],Ky[k],Iy);
					++k;
				}
			}
			if(Ix == 0.0 || Iy == 0.0){
				I_mag[position] = 0;
			} else {
				I_mag[k] = arm_sqrt_f32(Ix*Ix + Iy*Iy);
			}



			theta[position] = atanf(Ix / Iy);
			//theta[position] = (z - (pow(z,3)/3) + (pow(z,5)/5) - (pow(z,7)/7)) * RAD2DEG;//in deg// //using arm_mult_f32 would be a better idea right ?
			//
			//After testing, we see that using the atanf function gives precise results for a long processing time whereas the taylor serie takes less time but gives us a butchered result
			//
			}
		}

	// local maxima suppression : this process' purpose is to thin the edges by suppressing pixels next to high intensity pixels
	// given their gradient angle

	float i,j = 0;
	for(uint8_t x = XY_OFFSET_3x3; x < IM_LENGTH_PX-XY_OFFSET_3x3; ++x){
		for(uint8_t y = XY_OFFSET_3x3; y < IM_HEIGHT_PX-XY_OFFSET_3x3; ++y){
			position = x + (y * IM_LENGTH_PX);
			if((theta[position] <= 22.5 && theta[position] >= -22.5) || (theta[position] <= -157.5 && theta[position] >= 157.5)){
				i = I_mag[position+1];
				j = I_mag[position-1];
			} else if((theta[position] > 22.5 && theta[position] <= 67.5) || (theta[position] <= -112.5 && theta[position] > -157.5)){
				i = I_mag[position + IM_LENGTH_PX +1];
				j = I_mag[position + IM_LENGTH_PX -1];
			} else if((theta[position] > 67.5 && theta[position] <= 112.5) || (theta[position] <= -67.5 && theta[position] > -112.5)){
				i = I_mag[position + IM_LENGTH_PX ];
				j = I_mag[position - IM_LENGTH_PX ];
			} else if((theta[position] > 112.5 && theta[position] < 157.5) || (theta[position] < -22.5 && theta[position] >= -67.5)){
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
	        if (img_buffer[position] > HIGH_THRESHOLD) {
	        	img_temp_buffer[position] = HIGH_THRESHOLD;
	        } else if (img_buffer[position] > LOW_THRESHOLD) {
	            img_temp_buffer[position] = LOW_THRESHOLD;
	        } else {
	            img_temp_buffer[position] = 0;
	        }
	    }
	}

	// Edge tracking by hysteresis : weak pixels are transformed into strong pixels if and only if one is present around it

	for (uint8_t x = 0; x < IM_LENGTH_PX; x++) {
		for (uint8_t y = 0; y < IM_HEIGHT_PX; y++) {
			position = x + (y * IM_LENGTH_PX);
			if(img_temp_buffer[position] == LOW_THRESHOLD){
				if(img_temp_buffer[position-IM_LENGTH_PX-1] == HIGH_THRESHOLD || img_temp_buffer[position-IM_LENGTH_PX] == HIGH_THRESHOLD ||
						img_temp_buffer[position-IM_LENGTH_PX+1] == HIGH_THRESHOLD || img_temp_buffer[position-1] == HIGH_THRESHOLD ||
						img_temp_buffer[position+1] == HIGH_THRESHOLD || img_temp_buffer[position+IM_LENGTH_PX-1] == HIGH_THRESHOLD ||
						img_temp_buffer[position-IM_LENGTH_PX] == HIGH_THRESHOLD || img_temp_buffer[position+IM_LENGTH_PX+1] == HIGH_THRESHOLD)
					img_buffer[position] = 1;
				else if(img_temp_buffer[position] == HIGH_THRESHOLD)
				img_buffer[position] = 1;
			else img_buffer[position] = 0;
			}
		}
	//returns a pointer to a binary array
	}
return img_buffer;
}

