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
static double buffer_x[5]={0,0,0,0,0};
static double buffer_y[5]={0,0,0,0,0};

// ------ PUBLIC variable definitions -------------------------
//--------------------------------------------------------------
// FUNCTION DEFINITIONS
//--------------------------------------------------------------
/**
 * @brief internal sensor stop function
 */
double MAFilter_x(double data)
{
    buffer_x[4]=buffer_x[3];
    buffer_x[3]=buffer_x[2];
    buffer_x[2]=buffer_x[1];
    buffer_x[1]=buffer_x[0];
    buffer_x[0]=data;
    return (buffer_x[0]+buffer_x[1]+buffer_x[2]+buffer_x[3]+buffer_x[4])/5.0;
}
/**
 * @brief internal sensor stop function
 */
double MAFilter_y(double data)
{
    buffer_y[4]=buffer_y[3];
    buffer_y[3]=buffer_y[2];
    buffer_y[2]=buffer_y[1];
    buffer_y[1]=buffer_y[0];
    buffer_y[0]=data;
    return (buffer_y[0]+buffer_y[1]+buffer_y[2]+buffer_y[3]+buffer_y[4])/5.0;
}