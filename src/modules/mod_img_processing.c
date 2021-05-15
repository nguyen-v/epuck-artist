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

// e-puck 2 main processor headers

#include <camera/po8030.h>
#include "camera/dcmi_camera.h"

// Module headers

#include <mod_img_processing.h>
#include <mod_path.h>
#include <mod_communication.h>
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
#define XY_OFFSET_3x3      1

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

#define BLACK_THRESHOLD    60
#define BLUE_MIN_VALUE     45
#define GREEN_BLUE_DIFF    20
#define GREEN_COEFF        1.2f
#define RED_COEFF          1.5f

#define NUMBER_COLORS      3

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

static uint8_t *sobel_angle_state;
static float *I_mag;

static bool capture_thd_alive = false;
static bool process_thd_alive = false;

/*===========================================================================*/
/* Semaphores.                                                               */
/*===========================================================================*/

static BSEMAPHORE_DECL(sem_image_captured, TRUE);
static BSEMAPHORE_DECL(sem_capture_image, TRUE);

/*===========================================================================*/
/* Module thread pointers.                                                   */
/*===========================================================================*/

static thread_t* ptr_capture_image;
static thread_t* ptr_process_image;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief                       classifies a color from its RGB values
 * @param[in]   rgb             rgb_color struct containing rgb information
 * @return                      a color (white, black, red, green, blue)
 */
static uint8_t classify_color(rgb_color rgb)
{
	if (rgb.red > RED_COEFF*rgb.blue && rgb.red > RED_COEFF*rgb.green)
		return red;
	else if (abs((int16_t)rgb.blue - (int16_t)rgb.green) < GREEN_BLUE_DIFF
	         && rgb.green > GREEN_COEFF*rgb.red)
		return green;
	else if ((rgb.blue > rgb.red) && (rgb.blue > rgb.green)
	           && (rgb.blue > BLUE_MIN_VALUE))
		return blue;
	else if ((rgb.red + rgb.green + rgb.blue)/NUMBER_COLORS < BLACK_THRESHOLD)
		return black;
	else
		return white;
}

/**
 * @brief                       converts an rgb565 color to a grayscale image
 *                              and classifies image colors
 * @param[out]  color           pointer to buffer containing path color
 * @return                      none
 */
static void set_grayscale_filter_colors(uint8_t* color)
{
	rgb_color rgb;
//	uint8_t red_px, green_px, blue_px = 0;
	for (uint16_t i = 0; i < (IM_LENGTH_PX * IM_HEIGHT_PX)*2; i+=2){

		// extract RGB values and convert to range 0-255
		uint16_t rgb_565 = ((int16_t)img_buffer[i]  << 8) | img_buffer[i+1];
		rgb.red = (rgb_565 & RED_MASK) >> RGB_RED_POS;
		rgb.green = (rgb_565 & GREEN_MASK) >> RGB_GREEN_POS;
		rgb.blue = (rgb_565 & BLUE_MASK) << RGB_BLUE_POS;

		// classify pixel color (black, red, green, blue, background (white))
		color[i/2] = classify_color(rgb);

		// convert img_buffer to grayscale
		img_buffer[i/2] = LUMA_RED_COEFF*rgb.red + LUMA_GREEN_COEFF*rgb.green
		                  + LUMA_BLUE_COEFF*rgb.blue;
	}
}

/**
 * @brief                       Filters out obvious noise and smoothens the image.
 * @return                      none
 * @note                        A 5x5 Gaussian filter was chosen with a standard
 *                              deviation of 1.
 */
static void gaussian_filter(void)
{
	uint16_t pos = 0;
	for (uint8_t x = 0; x < IM_LENGTH_PX; ++x){
		for (uint8_t y = 0; y < IM_HEIGHT_PX; ++y){
			pos = position(x,y);
			if (x < XY_OFFSET_5x5 || x >= (IM_LENGTH_PX - XY_OFFSET_5x5)
			    || y < XY_OFFSET_5x5 || y >= (IM_HEIGHT_PX - XY_OFFSET_5x5)) {
				img_temp_buffer[pos] = img_buffer[pos];
				continue;
			}
			uint16_t conv = 0;
			uint16_t k = 0;
			for (int8_t x_ker = -XY_OFFSET_5x5; x_ker <= XY_OFFSET_5x5; ++x_ker){
				for (int8_t y_ker = -XY_OFFSET_5x5; y_ker <= XY_OFFSET_5x5; ++y_ker){
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
 * @brief                       The sobel filter emphasizes edges by computing
 *                              the norm of the gradient of the image intensity
 *                              for each pixel. From these values, we can also
 *                              extract the gradient's angle from this function
 *                              and use it later for edge thinning.
 * @return        max           The maximum gradient intensity computed for an image.
 */
static float sobel_filter(void)
{
	float max = 0;
	float theta = 0;
	uint16_t pos = 0;
	for (uint8_t x = XY_OFFSET_3x3; x < IM_LENGTH_PX-XY_OFFSET_3x3; ++x){
		for (uint8_t y = XY_OFFSET_3x3; y < IM_HEIGHT_PX-XY_OFFSET_3x3; ++y){
			pos = position(x,y);
			int16_t Ix = 10;
			int16_t Iy = 0;
			uint16_t k = 0;
			for (int8_t x_ker = -XY_OFFSET_3x3; x_ker <= XY_OFFSET_3x3; ++x_ker){
				for (int8_t y_ker = -XY_OFFSET_3x3; y_ker <= XY_OFFSET_3x3; ++y_ker){
					Ix += img_temp_buffer[pos + x_ker +(y_ker * IM_LENGTH_PX)]*Kx[k];
					Iy += img_temp_buffer[pos+ x_ker +(y_ker * IM_LENGTH_PX)]*Ky[k];
//					Ix = (int16_t)__SMMLA((int32_t)(img_temp_buffer[pos + x_ker +(y_ker * IM_LENGTH_PX)]), (int32_t)(Kx[k]), (int32_t)Ix);
//					Iy = (int16_t)__SMMLA((int32_t)(img_temp_buffer[pos + x_ker +(y_ker * IM_LENGTH_PX)]), (int32_t)(Ky[k]), (int32_t)Iy);
					++k;
				}
			}
			I_mag[pos] = sqrtf(Ix*Ix + Iy*Iy);
			if (I_mag[pos]>max)
				max = I_mag[pos];

			theta = atan2f((float)Ix , (float)Iy)*RAD2DEG;

			if ((theta > FIRST_OCTANT_L && theta <= FIRST_OCTANT_H)){
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
 * @brief                       Sets the color at the edge to the color inside
 *                              the shape that the edge is encircling.
 * @note                        The sobel_angle_state buffer tells us in which
 *                              octant the interior of a shape is, in relation
 *                              to the corresponding pixel
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
			if (img_buffer[pos] == STRONG_PIXEL) {
				switch (sobel_angle_state[pos]) {
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
 * @brief                       Depending on the gradient's angle, we know in which
 *                              direction an edge thickens for all pixels.
 *                              Given that, we check if a pixel's gradient intensity
 *                              is higher than both pixels located in its
 *                              corresponding octants. If it is the case, then
 *                              we keep its value. If not, then we put it to 0.
 * @param[in]      max          The maximum gradient intensity computed for an image.
 * @return                      none
 */
static void local_max_supression(float max)
{
	float mag_octant = 0;
	float mag_octant_opposed = 0;

	const int8_t dx = 1;
	const int8_t dy = IM_LENGTH_PX;

	uint16_t pos = 0;
	for (uint8_t x = XY_OFFSET_3x3; x < IM_LENGTH_PX-XY_OFFSET_3x3; ++x){
		for (uint8_t y = XY_OFFSET_3x3; y < IM_HEIGHT_PX-XY_OFFSET_3x3; ++y){
			pos = position(x,y);
			switch (sobel_angle_state[pos]){
				case first_octant:
				case fifth_octant:
					mag_octant = I_mag[pos - dx];
					mag_octant_opposed = I_mag[pos + dx];
					break;
				case second_octant:
				case sixth_octant:
					mag_octant = I_mag[pos - dy + dx];
					mag_octant_opposed = I_mag[pos + dy - dx];
					break;
				case third_octant:
				case seventh_octant:
					mag_octant = I_mag[pos - dy];
					mag_octant_opposed = I_mag[pos + dy];
				break;
				case fourth_octant:
				case eighth_octant:
					mag_octant = I_mag[pos - dy - dx];
					mag_octant_opposed = I_mag[pos + dy + dx];
				break;
			}
			if ((I_mag[pos] >= mag_octant) && (I_mag[pos] >= mag_octant_opposed)) {
				img_buffer[pos] = I_mag[pos]/max*STRONG_PIXEL;
			} else
				img_buffer[pos] = BG_PIXEL;
		}
	}
}

/**
 * @brief                       Compares the gradient intensity of all pixels to
 *                              selected threshold values and
 *                              separates them into 3 categories : Strong pixels,
 *                              Weak pixels and Background pixels.
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
 * @brief                       Checks if weak pixels are near a strong pixels.
 *                              If it is the case, then they are turned into strong pixels.
 *                              Otherwise, they are turned to background pixels.
 * @return                      none
 */
static void edge_track_hyst(void)
{
	const int8_t dx = 1;
	const int8_t dy = IM_LENGTH_PX;

	uint16_t pos = 0;
	for (uint8_t x = 1; x < IM_LENGTH_PX-1; x++) {
		for (uint8_t y = 1; y < IM_HEIGHT_PX-1; y++) {
			pos = position(x,y);
			if (img_temp_buffer[pos] == WEAK_PIXEL){
				if (img_temp_buffer[pos-dy-dx] == STRONG_PIXEL
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
			} else if (img_temp_buffer[pos] == STRONG_PIXEL) {
				img_buffer[pos] = STRONG_PIXEL;
			} else {
				img_buffer[pos] = BG_PIXEL;
			}
		}
	}
}

/**
 * @brief                       Fills the image with background pixels
 * @return                      none
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
 * @brief                       Removes removes image borders (width = MARGIN_PX)
 * @return                      none
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
 * @brief                        Removes pixels that don't have any neighbours
 * @return                       none
 */
static void remove_unique_px(void)
{
	const int8_t dx = 1;
	const int8_t dy = IM_LENGTH_PX;
	uint16_t pos= 0;
	for (uint8_t x = 1; x < IM_LENGTH_PX-1; x++) {
		for (uint8_t y = 1; y < IM_HEIGHT_PX-1; y++) {
			pos = position(x,y);
			if (img_buffer[pos]) {
				if (  !img_buffer[pos + dx]      && !img_buffer[pos - dx]
				   && !img_buffer[pos + dy]      && !img_buffer[pos - dy]
				   && !img_buffer[pos - dy - dx] && !img_buffer[pos - dy + dx]
				   && !img_buffer[pos + dy - dx] && !img_buffer[pos + dy + dx])
					img_buffer[pos] = BG_PIXEL;
			}
		}
	}
}


/**
 * @brief                         canny edge detection algorithm on img_buffer
 * @return                        none
 * @detaiL                        this algorithm performs canny edge detection
 *                                on an image buffer obtained by the camera.
 *                                The resulting buffers are a buffer of size
 *                                IM_LENGTH_PX * IM_HEIGHT_PX containing edges
 *                                of img_buffer. Active pixels take the value of
 *                                IM_MAX_VALUE and inactive pixels take a value of 0.
 */
static void canny_edge(void)
{
	// free position and color buffers
	data_free();


	uint8_t* color = data_alloc_color(IM_LENGTH_PX * IM_HEIGHT_PX);
	set_grayscale_filter_colors(color);

	// send grayscale image
	com_send_data((BaseSequentialStream *)&SD3, img_buffer,
	              IM_LENGTH_PX*IM_HEIGHT_PX*sizeof(uint8_t), MSG_IMAGE_GRAYSCALE);

	img_temp_buffer = calloc(IM_LENGTH_PX*IM_HEIGHT_PX, sizeof(uint8_t));
	gaussian_filter();

	// send image after Gaussian filter
	com_send_data((BaseSequentialStream *)&SD3, img_temp_buffer,
	              IM_LENGTH_PX*IM_HEIGHT_PX*sizeof(uint8_t), MSG_IMAGE_GAUSS);

	sobel_angle_state = calloc(IM_LENGTH_PX*IM_HEIGHT_PX, sizeof(uint8_t));
	I_mag = malloc(IM_LENGTH_PX*IM_HEIGHT_PX*sizeof(float));

	float max = sobel_filter();

	// send Sobel filter gradient data
	com_send_data((BaseSequentialStream *)&SD3, sobel_angle_state,
	              IM_LENGTH_PX*IM_HEIGHT_PX*sizeof(uint8_t), MSG_IMAGE_SOBEL_MAG);

	local_max_supression(max);

	// send image after local max suppression (local thresholding)
	com_send_data((BaseSequentialStream *)&SD3, img_buffer,
	              IM_LENGTH_PX*IM_HEIGHT_PX*sizeof(uint8_t), MSG_IMAGE_LOCAL_THR);

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

	// send final Canny filtered image
	com_send_data((BaseSequentialStream *)&SD3, img_buffer,
	              IM_LENGTH_PX*IM_HEIGHT_PX*sizeof(uint8_t), MSG_IMAGE_CANNY);

	free(img_temp_buffer);

	free(sobel_angle_state);
}

/*===========================================================================*/
/* Module threads.                                                           */
/*===========================================================================*/

static THD_WORKING_AREA(wa_capture_image, 256);
static THD_FUNCTION(thd_capture_image, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	po8030_advanced_config(FORMAT_RGB565,
	                       CAMERA_X_POS, CAMERA_Y_POS,
	                       CAMERA_SUBSAMPLING*IM_LENGTH_PX,
	                       CAMERA_SUBSAMPLING*IM_HEIGHT_PX,
	                       SUBSAMPLING_X4, SUBSAMPLING_X4);
	po8030_set_contrast(CAMERA_CONTRAST);
	po8030_set_awb(1);
	dcmi_disable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();
	chThdSleepMilliseconds(1000);
	while (1){
		chBSemWait(&sem_capture_image);
		dcmi_capture_start();
		wait_image_ready();
		chBSemSignal(&sem_image_captured);

    }
}


static THD_WORKING_AREA(wa_process_image, 1024);
static THD_FUNCTION(thd_process_image, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while (1){
		chBSemWait(&sem_image_captured);
		img_buffer = dcmi_get_last_image_ptr();

		// send rgb image
		com_send_data((BaseSequentialStream *)&SD3, img_buffer,
		              IM_LENGTH_PX*IM_HEIGHT_PX*sizeof(uint16_t), MSG_IMAGE_RGB);
		canny_edge();
		path_planning();
	}
}

static void capture_create_thd(void)
{
	if (!capture_thd_alive) {
		ptr_capture_image = chThdCreateStatic(wa_capture_image, sizeof(wa_capture_image),
		                                      NORMALPRIO, thd_capture_image, NULL);
		capture_thd_alive = true;
	}
}

static void process_img_create_thd(void)
{
	if (!process_thd_alive) {
		ptr_process_image = chThdCreateStatic(wa_process_image, sizeof(wa_process_image),
		                                      NORMALPRIO+1, thd_process_image, NULL);
		process_thd_alive = true;
	}
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

uint8_t* get_img_buffer(void)
{
	return img_buffer;
}



void mod_img_processing_init(void)
{
	capture_create_thd();
	process_img_create_thd();
}

void capture_image(void)
{
	chBSemSignal(&sem_capture_image);
}
