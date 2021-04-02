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
/* Module exported functions.                                                   */
/*===========================================================================*/

/**
 * @brief	Starts serial communication.
 * @note	UART3 is connected to COM8 (Bluetooth).
 */
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


/**
 * @brief			Reads a command from the computer.
 * @note			UART3 is connected to COM8 (Bluetooth).
 *
 * @param[in] in 	Pointer to a @p BaseSequentialStream or derived class
 * @return			An utf-8 encoded command
 */
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


/**
 * @brief				Reads movement data from the computer.
 * @note				UART3 is connected to COM8 (Bluetooth).
 *
 * @param[in] 	in 		Pointer to a @p BaseSequentialStream or derived class
 * @param[out] 	pos		Pointer to position buffer @p cartesian_coord
 * @return				Length of the position buffer.
 */
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

	chprintf((BaseSequentialStream *)&SDU1, "Data detected \r \n");

	// reset data information and free buffers
	data_free();

	// get length of incoming data
	c1 = chSequentialStreamGet(in);
	c2 = chSequentialStreamGet(in);
	length = (uint16_t)((c1 | c2<<8));

	// save length and perform verifications
	data_set_length(length);
	length = data_get_length();

	chprintf((BaseSequentialStream *)&SDU1, "Length = %d \r \n", length);

	// get pointers to position and color buffers
	cartesian_coord* pos = data_alloc_xy(length);
	uint8_t* color = data_alloc_color(length);

	if (pos == NULL || color == NULL) {
		chprintf((BaseSequentialStream *)&SDU1, "Allocation failed \r \n");
		return 0;
	}
	chprintf((BaseSequentialStream *)&SDU1, "Position and color buffers allocated \r \n");
	chprintf((BaseSequentialStream *)&SDU1, "Size in bytes of color buffer = %d \r \n", length*sizeof(uint8_t));
	chprintf((BaseSequentialStream *)&SDU1, "Size in bytes of position buffer = %d \r \n", length*sizeof(cartesian_coord));

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
	}

	chprintf((BaseSequentialStream *)&SDU1, "Position and color buffers filled");

	for(uint16_t i = 0; i < length; ++i) {
		chprintf((BaseSequentialStream *)&SDU1, "size=%d \r \n", length);
		chprintf((BaseSequentialStream *)&SDU1, "c=%d \r \n", color[i]);
		chprintf((BaseSequentialStream *)&SDU1, "x=%d \r \n", pos[i].x);
		chprintf((BaseSequentialStream *)&SDU1, "y=%d \r \n", pos[i].y);
	}

	return length;
}





