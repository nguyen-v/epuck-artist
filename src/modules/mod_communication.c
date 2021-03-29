/**
 * @file    mod_communication.c
 * @brief   E-puck 2 communication to computer.
 */

#include "include/mod_communication.h"

/**
 * @brief   Starts an ADC conversion.
 * @details Starts an asynchronous conversion operation.
 * @note    The buffer is organized as a matrix of M*N elements where M is the
 *          channels number configured into the conversion group and N is the
 *          buffer depth. The samples are sequentially written into the buffer
 *          with no gaps.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 * @param[in] grpp      pointer to a @p ADCConversionGroup object
 * @param[out] samples  pointer to the samples buffer
 * @param[in] depth     buffer depth (matrix rows number). The buffer depth
 *                      must be one or an even number.
 *
 * @api
 */
