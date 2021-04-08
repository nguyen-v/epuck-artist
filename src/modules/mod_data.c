/**
 * @file    mod_data.c
 * @brief   Module for memory allocation of data.
 * @note
 */

// C standard header files

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

// Module headers

#include <mod_data.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define MAX_ALLOCATED_DATA	100000 // max size in bytes for data structures
#define SIZE_OF_DATA		sizeof(cartesian_coord) + sizeof(uint8_t)
#define MAX_LENGTH			MAX_ALLOCATED_DATA/SIZE_OF_DATA

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static cartesian_coord* pos = NULL;
static uint8_t* color = NULL;	// not in cartesian_coord to avoid padding
static uint16_t data_length = 0;

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

cartesian_coord* data_get_pos(void)
{
	return pos;
}


void data_set_length(uint16_t length)
{
	if(length > MAX_LENGTH) {
		data_length = MAX_LENGTH;
	} else {
		data_length = length;
	}
}

uint16_t data_get_length(void)
{
	return data_length;
}


void data_free(void)
{
	free(pos);
	free(color);
	data_length = 0;
	pos = NULL;
	color = NULL;
}


cartesian_coord* data_alloc_xy(uint16_t length)
{
	uint16_t temp_length = length;

	if(length > MAX_LENGTH) {
		temp_length = MAX_LENGTH;
	}

	pos = (cartesian_coord*)malloc(temp_length*sizeof(cartesian_coord));
	if(pos == NULL) {
		return pos;
	}

	return pos;
}


uint8_t* data_alloc_color(uint16_t length)
{
	uint16_t temp_length = length;
	if(length > MAX_LENGTH) {
		temp_length = MAX_LENGTH;
	}

	color = (uint8_t*)malloc(temp_length*sizeof(uint8_t));

	if(color == NULL) {
		return color;
	}

	return color;
}
