/**
 * @file    mod_data.h
 * @brief   Data structures and data related constants.
 */

#ifndef _MOD_DATA_H_
#define _MOD_DATA_H_

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef struct cartesian_coord {
	uint16_t x;
	uint16_t y;
} cartesian_coord;

enum Colors{white, black, red, green, blue};

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/**
 * @brief				Returns pointer to position buffer
 * @param				none
 * @return				pointer to position buffer
 */
cartesian_coord* data_get_pos(void);

/**
 * @brief				Returns pointer to color buffer
 * @param				none
 * @return				pointer to color buffer
 */
uint8_t* data_get_color(void);

/**
 * @brief				Sets the length (number of coordinates)
 * @param[in]			Number of coordinates
 * @return				none
 */
void data_set_length(uint16_t length);

/**
 * @brief				Returns the length of position/color buffers
 * 						(number of coordinates)
 * @param				none
 * @return				Number of coordinates
 */
uint16_t data_get_length(void);

/**
 * @brief				Resets color and position buffer information
 * @param				none
 * @return				none
 */
void data_free(void);

void data_free_pos(void);

void data_free_color(void);


/**
 * @brief				Allocates memory for the position buffer.
 *
 * @param[in] 	length 	Length (number of coordinates)
 * @return				Pointer to position buffer.
 * 						NULL if allocation failed.
 */
cartesian_coord* data_alloc_xy(uint16_t length);

/**
 * @brief				Allocates memory for the color buffer.
 *
 * @param[in] 	length 	Length (number of coordinates)
 * @return				Pointer to color buffer.
 * 						NULL if allocation failed.
 */
uint8_t* data_alloc_color(uint16_t length);

/**
 * @brief				Rellocates memory for the color buffer.
 *
 * @param[in] 	length 	Length (number of coordinates)
 * @return				Pointer to color buffer.
 * 						NULL if allocation failed.
 */
uint8_t* data_realloc_color(uint16_t length);

#endif
