/**
 * @file    mod_state.h
 * @brief   External declarations for state module.
 */

#ifndef _MOD_STATE_H_
#define _MOD_STATE_H_

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/**
 * @brief               Creates command processing thread for receiving commands
 *                      from the computer.
 * @return              none
 */
void create_thd_process_cmd(void);


#endif /* _MOD_STATE_H_ */
