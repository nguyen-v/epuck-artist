/**
 * @file    mod_communication.h
 * @brief   E-puck 2 communication to PC header.
 */

#ifndef _MOD_COMMUNICATION_H_
#define _MOD_COMMUNICATION_H_

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define SERIAL_BIT_RATE			115200

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

void com_serial_start(void);

uint8_t com_receive_command(BaseSequentialStream* in);

uint16_t com_receive_data(BaseSequentialStream* in);

#endif
