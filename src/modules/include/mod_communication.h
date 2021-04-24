/**
 * @file    mod_communication.h
 * @brief   E-puck 2 communication to PC header.
 */

#ifndef _MOD_COMMUNICATION_H_
#define _MOD_COMMUNICATION_H_

/*===========================================================================*/
/* External mutexes, semaphores.                                             */
/*===========================================================================*/

//extern mutex_t serial_mtx;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/**
 * @brief	Starts serial communication.
 * @note	UART3 is connected to COM8 (Bluetooth).
 */
void com_serial_start(void);

/**
 * @brief	Returns true if e-puck is currently reading data.
 * @return	Bool indicating reading state of e-puck.
 */
bool data_ready(void);

/**
 * @brief			Reads a command from the computer.
 * @note			UART3 is connected to COM8 (Bluetooth).
 *
 * @param[in] in 	Pointer to a @p BaseSequentialStream or derived class
 * @return			An utf-8 encoded command
 */
uint8_t com_receive_command(BaseSequentialStream* in);

/**
 * @brief			Reads length data (uint8) from the computer.
 * @note			UART3 is connected to COM8 (Bluetooth).
 *
 * @param[in] in 	Pointer to a @p BaseSequentialStream or derived class
 * @return			Length in mm
 */
uint8_t com_receive_length(BaseSequentialStream* in);

/**
 * @brief				Reads position and color data from the computer and
 * 						and fills corresponding buffers.
 * @note				UART3 is connected to COM8 (Bluetooth).
 *
 * @param[in] 	in 		Pointer to a @p BaseSequentialStream or derived class
 * @param[out] 	pos		Pointer to position buffer @p cartesian_coord
 * @return				Length of the position buffer.
 */
uint16_t com_receive_data(BaseSequentialStream* in);

#endif
