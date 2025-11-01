/********************************************************************************
 * @file    interpolate.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef INTERPOLATE_H
#define INTERPOLATE_H

/********************************************************************************
 * Includes
 ********************************************************************************/

/********************************************************************************
 * Typedefs / Enums / Structs
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
float get_float_index(float input, const float *array, int max_idx, bool is_ascending);
float get_interpolated_value(float idx, const float *array, int max_idx);
float get_2d_interpolated_value(const float *array, int max_rows, int max_cols, float pix_idx, float x_idx);
int find_floor_index(float x, float *arr, int arr_len);

#endif // INTERPOLATE_H
