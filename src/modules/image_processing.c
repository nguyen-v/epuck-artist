#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>


#define IM_LENGTH_PX 100
#define IM_HEIGHT_PX 90
#define XY_OFFSET_5x5 2
#define XY_OFFSET_3x3 1

// The weights of the gaussian function were directly taken from the internet,
// Would it be wise to test it for different values of the standard deviation ?
// This will mainly affect the blur's effect and thus the final quality of the image...//
#define KER_DIV 115
const int8_t Gaus5x5 = { 2,  4,  5,  4, 2,
                         4,  9, 12,  9, 4,
                         5, 12, 15, 12, 5,
                         4,  9, 12,  9, 4,
                         2,  4,  5,  4, 2 };

const int8_t Kx = {-1, 0, 1,
				   -2, 0, 2,
				   -1, 0, 1 };

const int8_t Ky = {1, 2, 1,
				   0, 0, 0,
				   -1, -2, -1};


//void im_acquisition(){
//
//		po8030_advanced_config(FORMAT_RGB565, 0, 10, IM_LENGTH_PX, IM_HEIGHT_PX, SUBSAMPLING_X1, SUBSAMPLING_X1);
//		dcmi_enable_double_buffering();
//		dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
//		dcmi_prepare();
//
//	    while(1){
//
//	    	systime_t time;
//	    	chThdSleepMilliseconds(12);
//
//	    	time = chVTGetSystemTime();
//	        //starts a capture
//			dcmi_capture_start();
//			//waits for the capture to be done
//			wait_image_ready();
//			//signals an image has been captured
//			chBSemSignal(&image_ready_sem);
//			chprintf((BaseSequentialStream *)&SDU1, "Capture_time_=%-7d \r\n",
//					chVTGetSystemTime()-time);
//
//	    }
//	}



// IMPORTANT / MAGIC NUMBERS HAVE TO BE DEFINED //


void canny_edge(){

	//image conversion from RGB to grayscale//
	uint8_t img_buffer = dcmi_get_last_image_ptr();
	uint16_t red, blue, green = 0;
	for(uint8_t i; i < IM_LENGTH_PX * IM_HEIGHT_PX; i+=2){
		red = (uint8_t)img_buffer[i/2] & 0xF8;
		green = (uint8_t)((img_buffer[i/2] & 0x07) || (img_buffer[i/2+1] & 0xe0));
		blue = (uint8_t)img_buffer[i/2+1] & 0x1F;
		img_buffer[i/2] = (0.2126 * red) + (0.7152 * green / 2.0) + (0.0722 * blue);
	}

	// 5x5 Gaussian Filter. A gaussian function is convoluted to the signal
	// in order to reduce the noise (Kernel convolution was used)//

	for(uint8_t x = XY_OFFSET_5x5; x < IM_LENGTH_PX-XY_OFFSET_5x5; ++x){
		for(uint8_t y = XY_OFFSET_5x5; y < IM_HEIGHT_PX-XY_OFFSET_5x5; ++y){
			uint16_t position = x + (y * IM_LENGTH_PX);

			uint16_t conv = 0;
			uint16_t k = 0;
			for(uint8_t x_ker = -XY_OFFSET_5x5; x_ker > XY_OFFSET_5x5; ++x_ker){
				for(uint8_t y_ker = -XY_OFFSET_5x5; y_ker > XY_OFFSET_5x5; ++y_ker){
//					conv += img_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)]*Gaus5x5[k];
					__SMMLA(img_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)],Gaus5x5[k],conv);
					++k;
				}
			}
			img_buffer[position] = (uint8_t)(conv / KER_DIV);  //Not sure if this cast is a good idea//
		}
	}
	// Application of the Sobel filters onto the image. //
	// It will gives us the gradient intensity. //

	for(uint8_t x = XY_OFFSET_3x3; x < IM_LENGTH_PX-XY_OFFSET_3x3; ++x){
		for(uint8_t y = XY_OFFSET_3x3; y < IM_HEIGHT_PX-XY_OFFSET_3x3; ++y){
			uint16_t position_x = x + (y * IM_LENGTH_PX);

			uint16_t Ix = 0;
			uint16_t Iy = 0;
			uint16_t k = 0;
			for(uint8_t x_ker = -XY_OFFSET_3x3; x_ker > XY_OFFSET_3x3; ++x_ker){
				for(uint8_t y_ker = -XY_OFFSET_3x3; y_ker > XY_OFFSET_3x3; ++y_ker){
//					Ix += img_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)]*Kx[k];
//					Iy += img_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)]*Ky[k];
					__SMMLA(img_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)],Kx[k],Ix);
					__SMMLA(img_buffer[position + x_ker +(y_ker * IM_LENGTH_PX)],Ky[k],Iy);
					++k;
				}
			}
			float Imag = arm_sqrt_f32(Ix*Ix + Iy*Iy);

		}
	}
}


