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

void com_serial_start(void);

bool data_ready(void);

uint8_t com_receive_command(BaseSequentialStream* in);

uint16_t com_receive_data(BaseSequentialStream* in);

#endif
