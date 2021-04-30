/**
 * @file    mod_img_processing.c
 * @brief   Handles the capture and the processing of an image.
 */

// C standard header files
#include <math.h>
#include <stdlib.h>

// ChibiOS headers
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <camera/po8030.h>
#include "camera/dcmi_camera.h"

#include <arm_math.h>

//#include <integer.h>

// Module headers
#include <mod_img_processing.h>
#include <mod_data.h>


/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define IM_LENGTH_PX 		100
#define IM_HEIGHT_PX 		90
#define XY_OFFSET_5x5 		2
#define XY_OFFSET_3x3 		1
#define RAD2DEG 			180./M_PI
#define DEG2RAD 			M_PI/180.
#define COEFF 				0.5f

// Those threshold values were chosen arbitrarily, more values should be tested if possible
// After testing, I noticed that low resolution images have a LOT of noise remaining in the final image.
// The threshold values should be adjusted in order to reduce it to the maximum of our abilities

#define HIGH_THRESHOLD 		0.2
#define LOW_THRESHOLD 		0.1

// if the maximum of all pixels is under this value,
// the picture is considered pitch black
#define MIN_LUMINANCE 		50.0

// color masks for RGB565 image format
#define RED_MASK			0xF800
#define GREEN_MASK			0x7E0
#define BLUE_MASK			0x1F
#define RGB_MAX_VALUE	    255.0f



// The weights of the gaussian function were directly taken from the internet,
// Would it be wise to test it for different values of the standard deviation ?
// This will mainly affect blur and thus the final quality of the image...//

#define KER_DIV 159
const uint8_t Gaus5x5[] = {	2,	4,	5,	4,	2,
                        	4,  9, 	12,	9, 	4,
							5, 	12, 15, 12, 5,
							4,  9, 	12,	9, 	4,
							2,  4,  5,  4, 	2};

const int8_t Kx[] =	{ -1,  0,  1,
				   	  -2,  0,  2,
					  -1,  0,  1 };

const int8_t Ky[] = { 1,  2,   1,
				   	   0,  0,  0,
					  -1, -2, -1};

static uint8_t *img_buffer;
static uint8_t *img_temp_buffer;

static uint8_t* sobel_angle_state;
static float* I_mag;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/
//(from https://en.wikipedia.org/wiki/HSL_and_HSV)
static float rgb_get_hue(uint8_t red, uint8_t green, uint8_t blue)
{
	float r = red/RGB_MAX_VALUE;
	float g = green/RGB_MAX_VALUE;
	float b = blue/RGB_MAX_VALUE;
	float max = fmax(fmax(r, g), b);
	float min = fmin(fmin(r, g), b);
	float c = max - min;	// chroma
	float H;
	if (c == 0)
		return 0;
	if(max == r)
		H = fmod((g-b)/c, 6);
	else if (max == g)
		H = (b-r)/c+2;
	else if (max == b)
		H = (r-g)/c+4;
	else
		return 0;

	H *= 60.;
//	return H < 0 ? H + 360 : H;
	return H;

}

static float rgb_get_luma(uint8_t red, uint8_t green, uint8_t blue)
{
	float r = red/RGB_MAX_VALUE;
	float g = green/RGB_MAX_VALUE;
	float b = blue/RGB_MAX_VALUE;
	return (0.2989 * (float)r) + (0.5870 * (float)g) + (0.1140 * (float)b);
}

static float rgb_get_sat(float luma, uint8_t red, uint8_t green, uint8_t blue)
{
	if (luma == 1 || luma == 0)
		return 0;
	float r = red/RGB_MAX_VALUE;
	float g = green/RGB_MAX_VALUE;
	float b = blue/RGB_MAX_VALUE;
	float max = fmax(fmax(r, g), b);
	float min = fmin(fmin(r, g), b);
	float c = max - min;	// chroma

	return c/(1-fabs(2*luma-1));
}

// IMPORTANT / MAGIC NUMBERS HAVE TO BE DEFINED //

/**
 * @brief	canny edge detection algorithm on img_buffer
 * @detail	this algorithm performs canny edge detection on an image buffer
 * 			obtained by the camera. The resulting buffers are a buffer of
 * 			size IM_LENGTH_PX * IM_HEIGHT_PX containing edges of img_buffer.
 * 			Active pixels take the value of IM_MAX_VALUE and inactive pixels
 * 			take a value of 0.
 */
static void canny_edge(void){

	// free position and color buffers
	data_free();
	uint8_t* color = data_alloc_color(IM_LENGTH_PX * IM_HEIGHT_PX);

	// image conversion from RGB to grayscale + saves the color for each pixel in a separate array
	uint8_t red_px, blue_px, green_px = 0;
	uint16_t low_threshold[3] = {0};
	float average[3] ={0};
	float hue_av = 0;
	float sat_av = 0;
	float lum_av = 0;
	for(uint16_t i = 0; i < (IM_LENGTH_PX * IM_HEIGHT_PX)*2; i+=2){
		uint16_t rgb565 = ((int16_t)img_buffer[i]  << 8) | img_buffer[i+1];
		// extract color bits
		red_px = (rgb565 & RED_MASK) >> 11;		// 0-31
		green_px = (rgb565 & GREEN_MASK) >> 5;	// 0-63
		blue_px = (rgb565 & BLUE_MASK);				// 0-31

		// convert to range 0-255 for each color
		red_px <<= 3;
		green_px <<= 2;
		blue_px <<= 3;

		// convert to HSL color space
		hsl_color hsl;
		hsl.hue = rgb_get_hue(red_px, green_px, blue_px);
		hsl.lum = rgb_get_luma(red_px, green_px, blue_px);
		hsl.sat = rgb_get_sat(hsl.lum, red_px, green_px, blue_px);

		hue_av += hsl.hue;
		sat_av += hsl.sat;
		lum_av += hsl.lum;

		average[0] += red_px;
		average[1] += green_px;
		average[2] += blue_px;
		}
		hue_av /= IM_LENGTH_PX*IM_HEIGHT_PX;
		sat_av /= IM_LENGTH_PX*IM_HEIGHT_PX;
		lum_av /= IM_LENGTH_PX*IM_HEIGHT_PX;


		average[0] /= IM_LENGTH_PX*IM_HEIGHT_PX;
		average[1] /= IM_LENGTH_PX*IM_HEIGHT_PX;
		average[2] /= IM_LENGTH_PX*IM_HEIGHT_PX;

		chprintf((BaseSequentialStream *)&SDU1, " av red %f\r\n", average[0]);
		chprintf((BaseSequentialStream *)&SDU1, " av green %f\r\n", average[1]);
		chprintf((BaseSequentialStream *)&SDU1, " av blue %f\r\n", average[2]);
		chprintf((BaseSequentialStream *)&SDU1, " av hue %f\r\n", hue_av);
		chprintf((BaseSequentialStream *)&SDU1, " av sat %f\r\n", sat_av);
		chprintf((BaseSequentialStream *)&SDU1, " av lum %f\r\n\n", lum_av);
	for(uint8_t i=0; i < 3; ++i)
		low_threshold[i] = (uint16_t)(COEFF*average[i]);

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
	// in order to reduce the noise (Kernel convolution was used)

	img_temp_buffer = calloc(IM_LENGTH_PX*IM_HEIGHT_PX, sizeof(uint8_t));
	uint16_t position = 0;
	for(uint8_t x = 0; x < IM_LENGTH_PX; ++x){
		for(uint8_t y = 0; y < IM_HEIGHT_PX; ++y){
			position = x + (y * IM_LENGTH_PX);
			if (x < XY_OFFSET_5x5 || x >= (IM_LENGTH_PX - XY_OFFSET_5x5) || y < XY_OFFSET_5x5 || y >= (IM_HEIGHT_PX - XY_OFFSET_5x5)) {
				img_temp_buffer[position] = img_buffer[position];
				continue;
			}
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

	// Application of the Sobel filters onto the image.
	// It will gives us the gradient intensity.

	sobel_angle_state = malloc(IM_LENGTH_PX*IM_HEIGHT_PX*sizeof(uint8_t));
	I_mag = calloc(IM_LENGTH_PX*IM_HEIGHT_PX, sizeof(float));

	float max = 0;
	float theta = 0;

	for(uint8_t x = XY_OFFSET_3x3; x < IM_LENGTH_PX-XY_OFFSET_3x3; ++x){
		for(uint8_t y = XY_OFFSET_3x3; y < IM_HEIGHT_PX-XY_OFFSET_3x3; ++y){
			position = x + (y * IM_LENGTH_PX);

			int16_t Ix = 0;
			int16_t Iy = 0;
			uint16_t k = 0;
			for(int8_t x_ker = -XY_OFFSET_3x3; x_ker <= XY_OFFSET_3x3; ++x_ker){
				for(int8_t y_ker = -XY_OFFSET_3x3; y_ker <= XY_OFFSET_3x3; ++y_ker){
					Ix += img_temp_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)]*Kx[k];
					Iy += img_temp_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)]*Ky[k];
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

			theta = atan2((float)Ix , (float)Iy);
			if((theta <= 22.5*DEG2RAD && theta >= -22.5*DEG2RAD) || (theta <= -157.5*DEG2RAD) || (theta >= 157.5*DEG2RAD)){
				sobel_angle_state[position] = 0;
			} else if((theta > 22.5*DEG2RAD && theta <= 67.5*DEG2RAD) || (theta <= -112.5*DEG2RAD && theta > -157.5*DEG2RAD)){
				sobel_angle_state[position] = 1;
			} else if((theta > 67.5*DEG2RAD && theta <= 112.5*DEG2RAD) || (theta <= -67.5*DEG2RAD && theta > -112.5*DEG2RAD)){
				sobel_angle_state[position] = 2;
			} else if((theta > 112.5*DEG2RAD && theta < 157.5*DEG2RAD) || (theta < -22.5*DEG2RAD && theta > -67.5*DEG2RAD)){
				sobel_angle_state[position] = 3;
			}
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
			switch(sobel_angle_state[position]){
				case 0:
					i = I_mag[position - 1];
					j = I_mag[position + 1];
					break;
				case 1:
					i = I_mag[position - IM_LENGTH_PX + 1];
					j = I_mag[position + IM_LENGTH_PX - 1];
					break;
				case 2:
					i = I_mag[position - IM_LENGTH_PX ];
					j = I_mag[position + IM_LENGTH_PX ];
				break;
				case 3:
					i = I_mag[position - IM_LENGTH_PX - 1 ];
					j = I_mag[position + IM_LENGTH_PX + 1];
				break;
			}
			// multiplied by constant >1 for thicker lines
			if((I_mag[position] >= i) && (I_mag[position] >= j))
				img_buffer[position] = (uint8_t)I_mag[position];
			else
				img_buffer[position] = 0;
		}
	}

	free(sobel_angle_state);
	free(I_mag);

	// Double threshold - This is used to identify pixel intensities and impose 2 pixel intensities
	if (max > MIN_LUMINANCE) {
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

		for (uint8_t x = 0; x < IM_LENGTH_PX; x++) {
			for (uint8_t y = 0; y < IM_HEIGHT_PX; y++) {
				position = x + (y * IM_LENGTH_PX);
				if(img_temp_buffer[position] == 100){
					if(img_temp_buffer[position-IM_LENGTH_PX-1] == 255|| img_temp_buffer[position-IM_LENGTH_PX] == 255 ||
							img_temp_buffer[position-IM_LENGTH_PX+1] == 255 || img_temp_buffer[position-1] == 255 ||
							img_temp_buffer[position+1] == 255 || img_temp_buffer[position+IM_LENGTH_PX-1] == 255 ||
							img_temp_buffer[position+IM_LENGTH_PX] == 255 || img_temp_buffer[position+IM_LENGTH_PX+1] == 255)
						img_buffer[position] = 255;
					else
						img_buffer[position] = 0;
				} else if(img_temp_buffer[position] == 255) {
					img_buffer[position] = 255;
				} else {
					img_buffer[position] = 0;
				}
			}
		}
	} else {
		for (uint8_t x = 0; x < IM_LENGTH_PX; x++) {
			for (uint8_t y = 0; y < IM_HEIGHT_PX; y++) {
				position = x + (y * IM_LENGTH_PX);
				img_buffer[position] = 0;
			}
		}
	}

	free(img_temp_buffer);

	// remove 3 px around image border (because blur results in white edges)
	for (uint8_t x = 0; x < IM_LENGTH_PX; x++) {
		img_buffer[x] = 0;
		img_buffer[x+IM_HEIGHT_PX] = 0;
		img_buffer[x+2*IM_HEIGHT_PX] = 0;
		img_buffer[x + (IM_HEIGHT_PX-1)*IM_LENGTH_PX] = 0;
		img_buffer[x + (IM_HEIGHT_PX-2)*IM_LENGTH_PX] = 0;
		img_buffer[x + (IM_HEIGHT_PX-3)*IM_LENGTH_PX] = 0;
	}

	for (uint8_t y = 0; y < IM_HEIGHT_PX; y++) {
		img_buffer[y*IM_LENGTH_PX] = 0;
		img_buffer[y*IM_LENGTH_PX+1] = 0;
		img_buffer[y*IM_LENGTH_PX+2] = 0;
		img_buffer[IM_LENGTH_PX-1 + y*IM_LENGTH_PX] = 0;
		img_buffer[IM_LENGTH_PX-2 + y*IM_LENGTH_PX] = 0;
		img_buffer[IM_LENGTH_PX-3 + y*IM_LENGTH_PX] = 0;
	}

	// remove isolated pixels
	for (uint8_t x = 2; x < IM_LENGTH_PX-2; x++) {
		for (uint8_t y = 2; y < IM_HEIGHT_PX-2; y++) {
				position = x + (y * IM_LENGTH_PX);
			if(img_buffer[position]) {
				uint8_t has_neighbour = 0;
				if(img_buffer[position + 1])
					++has_neighbour;
				if(img_buffer[position - 1])
					++has_neighbour;
				if(img_buffer[position + IM_LENGTH_PX])
					++has_neighbour;
				if(img_buffer[position - IM_LENGTH_PX])
					++has_neighbour;
				if(img_buffer[position - IM_LENGTH_PX - 1])
					++has_neighbour;
				if(img_buffer[position - IM_LENGTH_PX + 1])
					++has_neighbour;
				if(img_buffer[position + IM_LENGTH_PX - 1])
					++has_neighbour;
				if(img_buffer[position + IM_LENGTH_PX + 1])
					++has_neighbour;
				if(!has_neighbour)
					img_buffer[position] = 0;
			}
		}
	}

	// fill isolated holes
	for (uint8_t x = 3; x < IM_LENGTH_PX-3; x++) {
		for (uint8_t y = 3; y < IM_HEIGHT_PX-3; y++) {
			position = x + (y * IM_LENGTH_PX);
			if(!img_buffer[position]) {
				uint8_t number_neighbour = 0;
				if(img_buffer[position + 1])
					++number_neighbour;
				if(img_buffer[position - 1])
					++number_neighbour;
				if(img_buffer[position + IM_LENGTH_PX])
					++number_neighbour;
				if(img_buffer[position - IM_LENGTH_PX])
					++number_neighbour;
				if(number_neighbour == 4)
					img_buffer[position] = 255;
			}
		}
	}
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

uint8_t* get_img_buffer(void)
{
	return img_buffer;
}


void capture_image(void){

	po8030_advanced_config(FORMAT_RGB565, 200, 0, 4*IM_LENGTH_PX, 4*IM_HEIGHT_PX, SUBSAMPLING_X4, SUBSAMPLING_X4);
//	po8030_set_brightness(64);
	po8030_set_contrast(40);
	po8030_set_awb(1);
//	po8030_set_ae(1);
	dcmi_disable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	dcmi_capture_start();
	wait_image_ready();
	img_buffer = dcmi_get_last_image_ptr();
	canny_edge();
}

void send_image(void) {
	if(image_is_ready()) {
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

}

void send_image_half(void) {
	if(image_is_ready()) {
		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer, 4000);
		chThdSleepMilliseconds(400);
		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+4000, 4000);
		chThdSleepMilliseconds(400);
		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+8000, 1000);
		chThdSleepMilliseconds(400);
	//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_temp_buffer, 4000);
	//	chThdSleepMilliseconds(400);
	//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_temp_buffer+4000, 4000);
	//	chThdSleepMilliseconds(400);
	//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_temp_buffer+8000, 1000);
	//	chThdSleepMilliseconds(400);

	//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer, 4000);
	//	chThdSleepMilliseconds(400);
	//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+4000, 4000);
	//	chThdSleepMilliseconds(400);
	//	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+8000, 1000);
	//	chThdSleepMilliseconds(400);
	}
}


