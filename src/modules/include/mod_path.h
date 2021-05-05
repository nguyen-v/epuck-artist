/*
 * @file	mod_path.h
 * @brief	Path structures, related constants and external declarations.
 */

#include <mod_data.h>

#ifndef _MOD_PATH_H_
#define _MOD_PATH_H_

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef struct edge_pos{
	struct cartesian_coord pos;
	uint16_t index;
} edge_pos;

typedef struct edge_track{
	struct cartesian_coord pos;
	uint8_t label;
	bool is_extremity;	//line = 0, edge = 1;
	uint8_t color;
} edge_track;

enum edge_status{start = 0, end = 1, init = 2};

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/**
 * @brief             modifies position and color buffer in mod_data
 *                    with positions and colors that the robot has to follow
 * @return            none
 */
void path_planning(void);


#endif /* _MOD_PATH_H_ */

