/*------------------------------------------------------------*-
  Simple Moving Average Filter - header file
  (c) Minh-An Dao 2022
  version 1.00 - 07/05/2022
---------------------------------------------------------------
 * N=3
 --------------------------------------------------------------*/
#ifndef __MAFILTER_H
#define __MAFILTER_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// ------ Public constants ------------------------------------
# define int16_t short
# define int32_t int
# define int64_t long int
// ------ Public function prototypes --------------------------
/**
 * @brief MAFILTER init function (public)
 * will automatically send data through mqtt protocol
 */
double MAFilter_x(double data);
double MAFilter_y(double data);
// ------ Public variable -------------------------------------

#ifdef __cplusplus
}
#endif

#endif