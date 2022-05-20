/*------------------------------------------------------------*-
  Simple Moving Average Filter - source
  (c) Minh-An Dao 2022
  version 1.00 - 07/05/2022
---------------------------------------------------------------
 * N=3
 --------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>

// ------ Private constants -----------------------------------
// #define N           3
// ------ Private function prototypes -------------------------
// ------ Private variables -----------------------------------
/** @brief tag used for ESP serial console messages */
// static double buffer_x;
// static double buffer_y;
static double _err_measure = 0.05; // Measurement Uncertainty 
static double _err_estimate = 0.05; // Estimation Uncertainty
static double _q = 0.01;                   // Process Noise
// --- x
static double _current_estimate_x = 0;
static double _last_estimate_x = 0;
static double _kalman_gain_x = 0;
// --- y
static double _current_estimate_y = 0;
static double _last_estimate_y = 0;
static double _kalman_gain_y = 0;
// ------ PUBLIC variable definitions -------------------------
//--------------------------------------------------------------
// FUNCTION DEFINITIONS
//--------------------------------------------------------------
/**
 * @brief internal sensor stop function
 */
double KMFilter_x(double data)
{
    _kalman_gain_x = _err_estimate/(_err_estimate + _err_measure);
    _current_estimate_x = _last_estimate_x + _kalman_gain_x * (data - _last_estimate_x);
    _err_estimate =  (1.0 - _kalman_gain_x)*_err_estimate + fabs(_last_estimate_x-_current_estimate_x)*_q;
    _last_estimate_x=_current_estimate_x;

    return _current_estimate_x;
}
/**
 * @brief internal sensor stop function
 */
double KMFilter_y(double data)
{
    _kalman_gain_y = _err_estimate/(_err_estimate + _err_measure);
    _current_estimate_y = _last_estimate_y + _kalman_gain_y * (data - _last_estimate_y);
    _err_estimate =  (1.0 - _kalman_gain_y)*_err_estimate + fabs(_last_estimate_y-_current_estimate_y)*_q;
    _last_estimate_y=_current_estimate_y;

    return _current_estimate_y;
}