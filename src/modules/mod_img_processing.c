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


// Module headers
#include <mod_img_processing.h>
#include <mod_data.h>
#include <tools.h>


/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

// Buffer size
#define IM_LENGTH_PX       100
#define IM_HEIGHT_PX       90

// Convolution offsets
#define XY_OFFSET_5x5      2

#define MARGIN_PX          4

// Conversion coefficients
#define RAD2DEG            180./M_PI
#define DEG2RAD            M_PI/180.

// Octant limits in degree
#define FIRST_OCTANT_L     -22.5f
#define FIRST_OCTANT_H     22.5f

#define SECOND_OCTANT_L    22.5
#define SECOND_OCTANT_H    67.5

#define THIRD_OCTANT_L     67.5
#define THIRD_OCTANT_H     112.5

#define FOURTH_OCTANT_L    112.5
#define FOURTH_OCTANT_H    157.5

#define FIFTH_OCTANT_L     157.5
#define FIFTH_OCTANT_H     -157.5

#define SIXTH_OCTANT_L     -157.5
#define SIXTH_OCTANT_H     -112.5

#define SEVENTH_OCTANT_L   -112.5
#define SEVENTH_OCTANT_H   -67.5

#define EIGHTH_OCTANT_L    -67.5
#define EIGHTH_OCTANT_H    -22.5


#define HIGH_THRESHOLD     0.18
#define LOW_THRESHOLD      0.05

#define WEAK_PIXEL         100
#define BG_PIXEL           0

// If the maximum of all pixels in I_mag is under this value,
// the picture is considered pitch black
#define MIN_I_MAG          100.0

// Color masks for RGB565 image format
#define RED_MASK           0xF800
#define GREEN_MASK         0x7E0
#define BLUE_MASK          0x1F

#define RGB_RED_POS        8
#define RGB_GREEN_POS      3
#define RGB_BLUE_POS       3

#define RGB_MAX_VALUE      255.0f

#define LUMA_RED_COEFF     0.2989f
#define LUMA_GREEN_COEFF   0.5870f
#define LUMA_BLUE_COEFF	    0.1140f

#define LOW_LIGHT_THR      1.1f

#define BLACK_THRESHOLD    80
#define SAME_COLOR_THR     5

#define HEXAGON_ANGLE      60.f
#define H_PRIME_MAX	        6
#define NUMBER_COLORS      3

// Hue thresholds in degree (0-360)
#define HUE_MIN	            0
#define HUE_MAX            360

#define RED_THR_L          25
#define RED_THR_H          280

#define GREEN_THR_L        135
#define GREEN_THR_H        195

#define BLUE_THR_L         200
#define BLUE_THR_H         230

// Camera settings
#define CAMERA_CONTRAST    150
#define CAMERA_SUBSAMPLING 4
#define CAMERA_X_POS       ((PO8030_MAX_WIDTH-CAMERA_SUBSAMPLING*IM_LENGTH_PX)/2)
#define CAMERA_Y_POS       0


#define KER_DIV 159
const uint8_t Gaus5x5[] = { 2,  4,  5,  4,  2,
                            4,  9,  12, 9,  4,
                            5,  12, 15, 12, 5,
                            4,  9,  12, 9,  4,
                            2,  4,  5,  4,  2 };

const int8_t Kx[] = { -1,  0,  1,
                      -2,  0,  2,
                      -1,  0,  1 };

const int8_t Ky[] = { 1,  2,  1,
                      0,  0,  0,
                     -1, -2, -1 };

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static uint8_t *img_buffer;
static uint8_t *img_temp_buffer;

static uint8_t* sobel_angle_state;
static float* I_mag;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief                       returns hue in hsl format from rgb values
 * @param[in]   rgb             rgb_color struct containing rgb information
 * @return                      hue in range 0-360
 * @note                        https://en.wikipedia.org/wiki/HSL_and_HSV
 */
static float rgb_get_hue(rgb_color rgb)
{
	float r = rgb.red/RGB_MAX_VALUE;
	float g = rgb.green/RGB_MAX_VALUE;
	float b = rgb.blue/RGB_MAX_VALUE;
	float max = fmax(fmax(r, g), b);
	float min = fmin(fmin(r, g), b);
	float chroma = max - min;
	float H;
	if (chroma == 0)
		return chroma;
	if(max == r)
		H = fmod((g-b)/chroma, H_PRIME_MAX);
	else if (max == g)
		H = (b-r)/chroma+H_PRIME_MAX/NUMBER_COLORS;
	else if (max == b)
		H = (r-g)/chroma+2*H_PRIME_MAX/NUMBER_COLORS;
	else
		return HUE_MIN;

	H *= HEXAGON_ANGLE;
	return H;

}

/**
 * @brief                       returns luma in hsl format from rgb values
 * @param[in]   rgb             rgb_color struct containing rgb information
 * @return                      lumina in range 0-1
 * @note                        https://en.wikipedia.org/wiki/HSL_and_HSV
 */
static float rgb_get_luma(rgb_color rgb)
{
	float r = rgb.red/RGB_MAX_VALUE;
	float g = rgb.green/RGB_MAX_VALUE;
	float b = rgb.blue/RGB_MAX_VALUE;
	return LUMA_RED_COEFF*r + LUMA_GREEN_COEFF*g + LUMA_BLUE_COEFF*b;
}



///**
// * @brief                     returns saturation in hsl format from rgb values
// * @param[in]   rgb           rgb_color struct containing rgb information
// * @return                    saturation in range 0-1
// * @note                      https://en.wikipedia.org/wiki/HSL_and_HSV
// * @note                      not accurate for low light conditions
// */
//static float rgb_get_sat(float luma, uint8_t red, uint8_t green, uint8_t blue)
//{
//	if (luma == 1 || luma == 0)
//		return 0;
//	float r = red/RGB_MAX_VALUE;
//	float g = green/RGB_MAX_VALUE;
//	float b = blue/RGB_MAX_VALUE;
//	float max = fmax(fmax(r, g), b);
//	float min = fmin(fmin(r, g), b);
//	float c = max - min;	// chroma
//
//	return c/(1-fabs(2*luma-1));
//}



/**
 * @brief                       returns luma in hsl format from rgb values
 * @param[in]   rgb             rgb_color struct containing rgb information
 * @return                      lumina in range 0-1
 * @note                        https://en.wikipedia.org/wiki/HSL_and_HSV
 */
static uint8_t classify_color(hsl_color hsl, rgb_color rgb)
{
	if (abs((int16_t)rgb.red - (int16_t)rgb.blue) < SAME_COLOR_THR
        && abs((int16_t)rgb.red - (int16_t)rgb.green) < SAME_COLOR_THR
        && abs((int16_t)rgb.green - (int16_t)rgb.blue) < SAME_COLOR_THR) {
		if (((int16_t)rgb.red + (int16_t)rgb.green + (int16_t)rgb.blue)/NUMBER_COLORS
		      > BLACK_THRESHOLD)
			return white;
		else
			return black;
	}
	if (hsl.hue <= RED_THR_L || hsl.hue > RED_THR_H)
		return red;
	else if (hsl.hue > GREEN_THR_L && hsl.hue <= GREEN_THR_H)
		return green;
	else if (hsl.hue > BLUE_THR_L && hsl.hue <= BLUE_THR_H)
		return blue;

	return white;
}

/**
 * @brief						converts an rgb565 color to a grayscale image
 * @param[out]  color           pointer to buffer containing path color
 * @return                      none
 */
static void set_grayscale_filter_colors(uint8_t* color)
{
	hsl_color hsl;
	rgb_color rgb;
//	uint8_t red_px, green_px, blue_px = 0;
	for(uint16_t i = 0; i < (IM_LENGTH_PX * IM_HEIGHT_PX)*2; i+=2){

		// extract RGB values and convert to range 0-255
		uint16_t rgb_565 = ((int16_t)img_buffer[i]  << 8) | img_buffer[i+1];
		rgb.red = (rgb_565 & RED_MASK) >> RGB_RED_POS;
		rgb.green = (rgb_565 & GREEN_MASK) >> RGB_GREEN_POS;
		rgb.blue = (rgb_565 & BLUE_MASK) << RGB_BLUE_POS;

		// convert to hsl color space for easier manipulation
		hsl.hue = rgb_get_hue(rgb);
		hsl.lum = rgb_get_luma(rgb);

		// classify pixel color (black, red, green, blue, background (white))
		color[i/2] = classify_color(hsl, rgb);

		// convert img_buffer to grayscale
		img_buffer[i/2] = hsl.lum*RGB_MAX_VALUE;
	}
}

/**
 * @brief     Filters out obvious noise and smoothens the image.
 * @return    none
 * @note      A 5x5 Gaussian filter was chosen with a standard deviation of 1.
 */
static void gaussian_filter(void)
{
	uint16_t pos = 0;
	for(uint8_t x = 0; x < IM_LENGTH_PX; ++x){
		for(uint8_t y = 0; y < IM_HEIGHT_PX; ++y){
			pos = position(x,y);
			if (x < XY_OFFSET_5x5 || x >= (IM_LENGTH_PX - XY_OFFSET_5x5)
			    || y < XY_OFFSET_5x5 || y >= (IM_HEIGHT_PX - XY_OFFSET_5x5)) {
				img_temp_buffer[pos] = img_buffer[pos];
				continue;
			}
			uint16_t conv = 0;
			uint16_t k = 0;
			for(int8_t x_ker = -XY_OFFSET_5x5; x_ker <= XY_OFFSET_5x5; ++x_ker){
				for(int8_t y_ker = -XY_OFFSET_5x5; y_ker <= XY_OFFSET_5x5; ++y_ker){
					conv += img_buffer[pos + x_ker +(y_ker * IM_LENGTH_PX)]*Gaus5x5[k];
//					__SMMLA(img_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)],Gaus5x5[k],conv);
					++k;
				}
			}
			img_temp_buffer[pos] = (uint8_t)(conv / KER_DIV);
		}
	}
}

/**
 * @brief                       The sobel filter emphasizes edges by computing the norm of the gradient of the image intensity
 *                              for each pixel. From these values, we can also extract the gradient's angle from this function
 *                              and use it later for edge thinning.
 * @return        max           The maximum gradient intensity computed for an image.
 */
static float sobel_filter(void)
{
	float max = 0;
	float theta = 0;
	uint16_t pos = 0;
	for(uint8_t x = XY_OFFSET_3x3; x < IM_LENGTH_PX-XY_OFFSET_3x3; ++x){
		for(uint8_t y = XY_OFFSET_3x3; y < IM_HEIGHT_PX-XY_OFFSET_3x3; ++y){
			pos = position(x,y);
			int16_t Ix = 10;
			int16_t Iy = 0;
			uint16_t k = 0;
			for(int8_t x_ker = -XY_OFFSET_3x3; x_ker <= XY_OFFSET_3x3; ++x_ker){
				for(int8_t y_ker = -XY_OFFSET_3x3; y_ker <= XY_OFFSET_3x3; ++y_ker){
					Ix += img_temp_buffer[pos + x_ker +(y_ker * IM_LENGTH_PX)]*Kx[k];
					Iy += img_temp_buffer[pos+ x_ker +(y_ker * IM_LENGTH_PX)]*Ky[k];
//					Ix = (int16_t)__SMMLA((int32_t)(img_temp_buffer[pos + x_ker +(y_ker * IM_LENGTH_PX)]), (int32_t)(Kx[k]), (int32_t)Ix);
//					Iy = (int16_t)__SMMLA((int32_t)(img_temp_buffer[pos + x_ker +(y_ker * IM_LENGTH_PX)]), (int32_t)(Ky[k]), (int32_t)Iy);
					++k;
				}
			}
			I_mag[pos] = sqrt(Ix*Ix + Iy*Iy);
			if(I_mag[pos]>max)
				max = I_mag[pos];

			theta = atan2((float)Ix , (float)Iy)*RAD2DEG;

			if((theta > FIRST_OCTANT_L && theta <= FIRST_OCTANT_H)){
				sobel_angle_state[pos] = first_octant;
			} else if ((theta > SECOND_OCTANT_L && theta <= SECOND_OCTANT_H)){
				sobel_angle_state[pos] = second_octant;
			} else if ((theta > THIRD_OCTANT_L && theta <= THIRD_OCTANT_H)){
				sobel_angle_state[pos] = third_octant;
			} else if ((theta > FOURTH_OCTANT_L && theta <= FOURTH_OCTANT_H	)){
				sobel_angle_state[pos] = fourth_octant;
			} else if ((theta > FIFTH_OCTANT_L) || (theta <= FIFTH_OCTANT_H)){
				sobel_angle_state[pos] =  fifth_octant;
			} else if ((theta > SIXTH_OCTANT_L && theta <= SIXTH_OCTANT_H)){
				sobel_angle_state[pos] = sixth_octant;
			} else if ((theta > SEVENTH_OCTANT_L && theta <= SEVENTH_OCTANT_H)){
				sobel_angle_state[pos] = seventh_octant;
			} else if ((theta > EIGHTH_OCTANT_L && theta <= EIGHTH_OCTANT_H)){
				sobel_angle_state[pos] = eighth_octant;
			}
		}
	}
	return max;
}

/**
 * @brief                       Sets the color at the edge to the color inside the shape that the edge is encircling.
 *                              This is possible for the sobel_angle_state points to the interior of
 *                              a shape.
 * @param[out]     color        Pointer to buffer containing path color
 * @return                      none
 */
static void set_strong_pixel_colors(uint8_t* color)
{
	uint16_t pos = 0;
	int8_t dx = 0;
	int8_t dy = 0;
	for (uint8_t x = 1; x < IM_LENGTH_PX-1; ++x) {
		for (uint8_t y = 1; y < IM_HEIGHT_PX-1; ++y) {
			pos = position(x,y);
			if(img_buffer[pos] == STRONG_PIXEL) {
				switch(sobel_angle_state[pos]) {
				case first_octant:
					dx = 1; dy = 0;
					break;
				case second_octant:
					dx = 1; dy = -IM_LENGTH_PX;
					break;
				case third_octant:
					dx = 0; dy = -IM_LENGTH_PX;
					break;
				case fourth_octant:
					dx = -1; dy = -IM_LENGTH_PX;
					break;
				case fifth_octant:
					dx = -1; dy = 0;
					break;
				case sixth_octant:
					dx = -1; dy = IM_LENGTH_PX;
					break;
				case seventh_octant:
					dx = 0; dy = IM_LENGTH_PX;
					break;
				case eighth_octant:
					dx = 1; dy = IM_LENGTH_PX;
					break;
				}
				color[pos] = color[pos + dx + dy];
			}
		}
	}
}


/**
 * @brief                       Depending on the gradient's angle, we know in which direction an edge thickens for all pixels.
 *                              Given that, we check if a pixel's gradient intensity is higher than both pixels located in its
 *                              corresponding octants. If it is the case, then we keep its value. If not, then we put it to 0.
 * @param[in]      max          The maximum gradient intensity computed for an image.
 * @return                      none
 */
static void local_max_supression(float max)
{
	float i= 0;
	float j= 0;

	const int8_t dx = 1;
	const int8_t dy = IM_LENGTH_PX;

	uint16_t pos = 0;
	for(uint8_t x = XY_OFFSET_3x3; x < IM_LENGTH_PX-XY_OFFSET_3x3; ++x){
		for(uint8_t y = XY_OFFSET_3x3; y < IM_HEIGHT_PX-XY_OFFSET_3x3; ++y){
			pos = position(x,y);
			switch(sobel_angle_state[pos]){
				case first_octant:
				case fifth_octant:
					i = I_mag[pos - dx];
					j = I_mag[pos + dx];
					break;
				case second_octant:
				case sixth_octant:
					i = I_mag[pos - dy + dx];
					j = I_mag[pos + dy - dx];
					break;
				case third_octant:
				case seventh_octant:
					i = I_mag[pos - dy];
					j = I_mag[pos + dy];
				break;
				case fourth_octant:
				case eighth_octant:
					i = I_mag[pos - dy - dx];
					j = I_mag[pos + dy + dx];
				break;
			}
			// multiplied by constant >1 for thicker lines
			if((I_mag[pos] >= i) && (I_mag[pos] >= j)) {
				img_buffer[pos] = I_mag[pos]/max*STRONG_PIXEL;
			} else
				img_buffer[pos] = BG_PIXEL;
		}
	}
}

/**
 * @brief    Compares the gradient intensity of all pixels to selected threshold values and
 *           separates them into 3 categories : Strong pixels, Weak pixels and Background pixels.
 * @return   none
 */
static void double_threshold(void)
{
	uint16_t pos = 0;
	for (uint8_t x = 0; x < IM_LENGTH_PX; x++) {
		for (uint8_t y = 0; y < IM_HEIGHT_PX; y++) {
			pos = position(x,y);
			if (img_buffer[pos] > HIGH_THRESHOLD*STRONG_PIXEL) {
				img_temp_buffer[pos] = STRONG_PIXEL;
			} else if (img_buffer[pos] > LOW_THRESHOLD*STRONG_PIXEL) {
				img_temp_buffer[pos] = WEAK_PIXEL;
			} else {
				img_temp_buffer[pos] = BG_PIXEL;
			}
		}
	}
}


/**
 * @brief    Checks if weak pixels are near a strong pixels. If it is the case, then they are turned into strong pixels.
 * 			 Otherwise, they are turned to background pixels.
 * @return   none
 */
static void edge_track_hyst(void)
{
	const int8_t dx = 1;
	const int8_t dy = IM_LENGTH_PX;

	uint16_t pos = 0;
	for (uint8_t x = 1; x < IM_LENGTH_PX-1; x++) {
		for (uint8_t y = 1; y < IM_HEIGHT_PX-1; y++) {
			pos = position(x,y);
			if(img_temp_buffer[pos] == WEAK_PIXEL){
				if(img_temp_buffer[pos-dy-dx] == STRONG_PIXEL
				    || img_temp_buffer[pos-dy] == STRONG_PIXEL
				    || img_temp_buffer[pos-dy+dx] == STRONG_PIXEL
				    || img_temp_buffer[pos-dx] == STRONG_PIXEL
				    || img_temp_buffer[pos+dx] == STRONG_PIXEL
				    || img_temp_buffer[pos+dy-dx] == STRONG_PIXEL
				    || img_temp_buffer[pos+dy] == STRONG_PIXEL
				    || img_temp_buffer[pos+dy+dx] == STRONG_PIXEL)

					img_buffer[pos] = STRONG_PIXEL;
				else
					img_buffer[pos] = BG_PIXEL;
			} else if(img_temp_buffer[pos] == STRONG_PIXEL) {
				img_buffer[pos] = STRONG_PIXEL;
			} else {
				img_buffer[pos] = BG_PIXEL;
			}
		}
	}
}

/**
 * @brief   Fills the image with background pixels
 * @return  none
 */
static void fill_background(void)
{
	uint16_t pos = 0;
	for (uint8_t x = 0; x < IM_LENGTH_PX; x++) {
		for (uint8_t y = 0; y < IM_HEIGHT_PX; y++) {
			pos = position(x,y);
			img_buffer[pos] = BG_PIXEL;
		}
	}
}

/**
 * @brief   Removes removes image borders (width = MARGIN_PX)
 * @return  none
 */
static void remove_borders(void)
{
	uint16_t pos = 0;

	for (uint8_t x = 0; x < IM_LENGTH_PX; ++x) {

		// top pixels
		for (uint8_t y = 0; y < MARGIN_PX; ++y) {
			pos = position(x,y);
			img_buffer[pos] = BG_PIXEL;
		}

		// bottom pixels
		for (uint8_t y = IM_HEIGHT_PX-MARGIN_PX; y < IM_HEIGHT_PX; ++y) {
			pos = position(x,y);
			img_buffer[pos] = BG_PIXEL;
		}
	}

	for (uint8_t y = MARGIN_PX; y < IM_HEIGHT_PX - MARGIN_PX; ++y) {

		// left pixels
		for (uint8_t x = 0; x < MARGIN_PX; ++x) {
			pos = position(x,y);
			img_buffer[pos] = BG_PIXEL;
		}

		// right pixels
		for (uint8_t x = IM_LENGTH_PX - MARGIN_PX; x < IM_LENGTH_PX; ++x) {
			pos = position(x,y);
			img_buffer[pos] = BG_PIXEL;
		}
	}
}

/**
 * @brief   Removes pixels that don't have any neighbours
 * @return  none
 */
static void remove_unique_px(void)
{
	const int8_t dx = 1;
	const int8_t dy = IM_LENGTH_PX;
	uint16_t pos= 0;
	for (uint8_t x = 1; x < IM_LENGTH_PX-1; x++) {
		for (uint8_t y = 1; y < IM_HEIGHT_PX-1; y++) {
			pos = position(x,y);
			if(img_buffer[pos]) {
				if(   !img_buffer[pos + dx]      && !img_buffer[pos - dx]
				   && !img_buffer[pos + dy]      && !img_buffer[pos - dy]
				   && !img_buffer[pos - dy - dx] && !img_buffer[pos - dy + dx]
				   && !img_buffer[pos + dy - dx] && !img_buffer[pos + dy + dx])
					img_buffer[pos] = BG_PIXEL;
			}
		}
	}
}


/**
 * @brief	canny edge detection algorithm on img_buffer
 * @return  none
 * @detail	this algorithm performs canny edge detection on an image buffer
 *          obtained by the camera. The resulting buffers are a buffer of
 *          size IM_LENGTH_PX * IM_HEIGHT_PX containing edges of img_buffer.
 *          Active pixels take the value of IM_MAX_VALUE and inactive pixels
 *          take a value of 0.
 */
static void canny_edge(void){

	// free position and color buffers
	data_free();


	uint8_t* color = data_alloc_color(IM_LENGTH_PX * IM_HEIGHT_PX);
	set_grayscale_filter_colors(color);

	img_temp_buffer = calloc(IM_LENGTH_PX*IM_HEIGHT_PX, sizeof(uint8_t));
	gaussian_filter();

	sobel_angle_state = malloc(IM_LENGTH_PX*IM_HEIGHT_PX*sizeof(uint8_t));
	I_mag = malloc(IM_LENGTH_PX*IM_HEIGHT_PX*sizeof(float));

	float max = sobel_filter();

	local_max_supression(max);

	free(I_mag);

	if (max > MIN_I_MAG) {
		double_threshold();
		edge_track_hyst();
		remove_borders();
		remove_unique_px();
		set_strong_pixel_colors(color);

	} else {
		fill_background();
	}

	free(img_temp_buffer);

	free(sobel_angle_state);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

uint8_t* get_img_buffer(void)
{
	return img_buffer;
}


void capture_image(void){

	po8030_advanced_config(FORMAT_RGB565,
	                      (PO8030_MAX_WIDTH-CAMERA_SUBSAMPLING*IM_LENGTH_PX)/2, 0,
	                       CAMERA_SUBSAMPLING*IM_LENGTH_PX,
	                       CAMERA_SUBSAMPLING*IM_HEIGHT_PX,
	                       SUBSAMPLING_X4, SUBSAMPLING_X4);
	po8030_set_contrast(CAMERA_CONTRAST);
	po8030_set_awb(1);
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
//		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
//		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer, 4000);
//		chThdSleepMilliseconds(400);
//		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+4000, 4000);
//		chThdSleepMilliseconds(400);
//		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+8000, 1000);
//		chThdSleepMilliseconds(400);
//		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
//		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_temp_buffer, 4000);
//		chThdSleepMilliseconds(400);
//		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_temp_buffer+4000, 4000);
//		chThdSleepMilliseconds(400);
//		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_temp_buffer+8000, 1000);
//		chThdSleepMilliseconds(400);

		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer, 4000);
		chThdSleepMilliseconds(400);
		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+4000, 4000);
		chThdSleepMilliseconds(400);
		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)img_buffer+8000, 1000);
		chThdSleepMilliseconds(400);
	}
}


