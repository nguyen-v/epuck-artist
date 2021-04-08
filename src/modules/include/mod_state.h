/**
 * @file    mod_state.h
 * @brief   Header for state module.
 */

#ifndef _MOD_STATE_H_
#define _MOD_STATE_H_

typedef enum{
	ST_FINDING_HOME,
	ST_CALIBRATION,
	ST_CAPTURE,
	ST_READING,
	ST_DRAWING,
	ST_INTERACTIVE
} state_t;

void create_thd_process_cmd(void);


#endif
