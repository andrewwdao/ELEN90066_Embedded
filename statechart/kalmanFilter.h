/*------------------------------------------------------------*-
  Simple Moving Average Filter - header file
  (c) Minh-An Dao 2022
  version 1.00 - 07/05/2022
---------------------------------------------------------------
 * N=3
 --------------------------------------------------------------*/
#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// ------ Public constants ------------------------------------
// ------ Public function prototypes --------------------------
/**
 * @brief Kalman Filter update function (public)
 * super lazy version
 */
double KMFilter_x(double data);
double KMFilter_y(double data);
// ------ Public variable -------------------------------------

#ifdef __cplusplus
}
#endif

#endif