/**
 * @file    mod_communication.c
 * @brief   E-puck 2 communication to computer (e-puck side).
 * @note 	Data is received and sent in little endian
 */

// C standard header files

#include <stdint.h>

// ChibiOS headers

#include "hal.h"
#include "ch.h"
#include <usbcfg.h>		// usb debug messages
#include "chprintf.h" 	// usb debug messages

// Module headers

#include <mod_communication.h>
#include <mod_data.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define SERIAL_BIT_RATE			115200
#define MAX_BUFFER_SIZE			4000

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static bool data_is_ready = false;

/*===========================================================================*/
/* Module mutexes, semaphores.                                               */
/*===========================================================================*/
//MUTEX_DECL(serial_mtx);

/*===========================================================================*/
/* Module exported functions.                                                   */
/*===========================================================================*/

void com_serial_start(void)
{
	static SerialConfig ser_cfg = {
		SERIAL_BIT_RATE,
		0,
		0,
		0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

bool data_ready(void)
{
	return data_is_ready;
}

uint8_t com_receive_command(BaseSequentialStream* in)
{
	volatile uint8_t c;
	uint8_t state = 0;

	while(state != 3) {
		c = chSequentialStreamGet(in);

		switch(state) {
			case 0:
				if(c == 'C')
					state = 1;
				else
					state = 0;
			case 1:
				if(c == 'M')
					state = 2;
				else if(c == 'C')
					state = 1;
				else
					state = 0;
			case 2:
				if(c == 'D')
					state = 3;
				else if (c == 'C')
					state = 1;
				else
					state = 0;
		}
	}
	return c = chSequentialStreamGet(in); // parses the command
}

uint8_t com_receive_length(BaseSequentialStream* in)
{
	volatile uint8_t c;
	uint8_t state = 0;

	while(state != 3) {
		c = chSequentialStreamGet(in);

		switch(state) {
			case 0:
				if(c == 'L')
					state = 1;
				else
					state = 0;
			case 1:
				if(c == 'E')
					state = 2;
				else if(c == 'L')
					state = 1;
				else
					state = 0;
			case 2:
				if(c == 'N')
					state = 3;
				else if (c == 'L')
					state = 1;
				else
					state = 0;
		}
	}
//	chprintf((BaseSequentialStream *)&SDU1, "Length detected \r \n");
	return c = chSequentialStreamGet(in); // parses length
}


uint16_t com_receive_data(BaseSequentialStream* in)
{
	volatile uint8_t c1, c2;
	volatile uint16_t length = 0;
	uint8_t state = 0;

	while(state != 4) {
		c1 = chSequentialStreamGet(in);

		switch(state) {
			case 0:
				if(c1 == 'M')
					state = 1;
				else
					state = 0;
			case 1:
				if(c1 == 'O')
					state = 2;
				else if(c1 == 'M')
					state = 1;
				else
					state = 0;
			case 2:
				if(c1 == 'V')
					state = 3;
				else if(c1 == 'M')
					state = 1;
				else
					state = 0;
			case 3:
				if(c1 == 'E')
					state = 4;
				else if(c1 == 'M')
					state = 1;
				else
					state = 0;
		}
	}

//	chprintf((BaseSequentialStream *)&SDU1, "Data detected \r \n");

	// reset data information and free buffers
	data_free();
	data_is_ready = false;

	// get length of incoming data
	c1 = chSequentialStreamGet(in);
	c2 = chSequentialStreamGet(in);
	length = (uint16_t)((c1 | c2<<8));

	// save length and perform verifications
	data_set_length(length);
	length = data_get_length();

	// allocate and get pointers to position and color buffers
	cartesian_coord* pos = data_alloc_xy(length);;
	uint8_t* color = data_alloc_color(length);

	if (pos == NULL || color == NULL) {
		data_is_ready = false;
//		chprintf((BaseSequentialStream *)&SDU1, "Allocation failed \r \n");
		return 0;
	}

//	chprintf((BaseSequentialStream *)&SDU1, "Position and color buffers allocated, length = %d \r \n", length);
//	chprintf((BaseSequentialStream *)&SDU1, "Size in bytes of color buffer = %d \r \n", length*sizeof(uint8_t));
//	chprintf((BaseSequentialStream *)&SDU1, "Size in bytes of position buffer = %d \r \n", length*sizeof(cartesian_coord));

	//	fill the position and color buffers
	for(uint16_t i = 0; i < length; ++i) {
		c1 = chSequentialStreamGet(in);
		color[i] = c1;

		c1 = chSequentialStreamGet(in);
		c2 = chSequentialStreamGet(in);
		pos[i].x = (uint16_t)((c1 | c2<<8));

		c1 = chSequentialStreamGet(in);
		c2 = chSequentialStreamGet(in);
		pos[i].y = (uint16_t)((c1 | c2<<8));

//		chprintf((BaseSequentialStream *)&SDU1, "c=%d \r \n", color[i]);
//		chprintf((BaseSequentialStream *)&SDU1, "x=%d \r \n", pos[i].x);
//		chprintf((BaseSequentialStream *)&SDU1, "y=%d \r \n", pos[i].y);
	}

	data_is_ready = true;
	return length;
}


void com_send_data(BaseSequentialStream* out, uint8_t* data, uint16_t size, message_type msg_type)
{
	// send start message
	chSequentialStreamWrite(out, (uint8_t*)"START\r", 6);

	// send message type
	switch(msg_type) {
		case MSG_COLOR:
			chprintf(out, "color");
			break;
		case MSG_IMAGE_RGB:
			chprintf(out, "rgb");
			break;
		case MSG_IMAGE_GRAYSCALE:
			chprintf(out, "grayscale");
			break;
		case MSG_IMAGE_GAUSS:
			chprintf(out, "gauss");
			break;
		case MSG_IMAGE_SOBEL:
			chprintf(out, "sobel");
			break;
		case MSG_IMAGE_CANNY:
			chprintf(out, "canny");
			break;
		case MSG_IMAGE_PATH:
			chprintf(out, "path");
			break;
	}
	chprintf(out, "\n");

	// send message length
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));

	// send message body

	/** @note: 	buffer has a maximum size of around 4000-4500, which is why we send
	  * 		it in packets of MAX_BUFFER_SIZE
	  */

	uint16_t length = size;
	if (size > MAX_BUFFER_SIZE) {
		while (length > MAX_BUFFER_SIZE) {
			chSequentialStreamWrite(out, (uint8_t*)data, sizeof(uint8_t) * MAX_BUFFER_SIZE);
			length -= MAX_BUFFER_SIZE;
		}
	} else {
		chSequentialStreamWrite(out, (uint8_t*)data, sizeof(uint8_t) * length);
	}
}

void com_request_color(uint8_t col)
{
	uint8_t color = white;
	switch(col) {
		case white:
			color = 'W';
			break;
		case black:
			color = 'D';
			break;
		case red:
			color = 'R';
			break;
		case blue:
			color = 'B';
			break;
		case green:
			color = 'G';
			break;
	}
	com_send_data((BaseSequentialStream *)&SD3, &color, 1, MSG_COLOR);
}


