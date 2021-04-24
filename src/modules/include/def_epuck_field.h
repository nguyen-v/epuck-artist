/**
 * @file    def_epuck_field.h
 * @brief   Header for drawing module.
 */

#ifndef _DEF_EPUCK_FIELD_H_
#define _DEF_EPUCK_FIELD_H_

#define X_RESOLUTION			1024
#define PI						3.14159265358979f

// standby coordinates
#define X_DEFAULT 				(X_RESOLUTION/2)
#define Y_DEFAULT  				0

// e-puck
#define SPOOL_DISTANCE     		8.0f    // cm
#define SPOOL_DIAMETER     		1.2f 	 // cm
#define SPOOL_PERIMETER			(PI*SPOOL_DIAMETER) // cm
#define NSTEP_ONE_TURN			1000.0f

// field geometry
#define SUPPORT_DISTANCE 		87.0f // cm
#define MARGIN					(SPOOL_DISTANCE*2) // cm

#define CM_TO_STEP				(NSTEP_ONE_TURN/SPOOL_PERIMETER)
#define MM_TO_STEP 				(CM_TO_STEP/10.)

// in steps
#define SPOOL_DISTANCE_ST  		((uint16_t)(SPOOL_DISTANCE*CM_TO_STEP))
#define SUPPORT_DISTANCE_ST  	((uint16_t)(SUPPORT_DISTANCE*CM_TO_STEP))
#define MARGIN_ST 				((uint16_t)(MARGIN*CM_TO_STEP))

#define CART_TO_ST				((float)(SUPPORT_DISTANCE_ST-2*MARGIN_ST)/X_RESOLUTION)

#endif
