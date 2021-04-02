/**
 * @file    mod_data.h
 * @brief   Data structures and data related constants.
 */

#ifndef _MOD_DATA_H_
#define _MOD_DATA_H_

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define MAX_ALLOCATED_DATA		100000 // max size in bytes for data structures
#define MAX_LENGTH				MAX_ALLOCATED_DATA/(sizeof(cartesian_coord) + sizeof(uint8_t))

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef struct cartesian_coord{
	uint16_t x;
	uint16_t y;
} cartesian_coord;

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

uint16_t data_get_length(void);

void data_set_length(uint16_t length);

void data_free(void);

cartesian_coord* data_alloc_xy(uint16_t length);

uint8_t* data_alloc_color(uint16_t length);

#endif
