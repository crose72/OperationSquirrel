/********************************************************************************
 * @file    interpolate.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Utilities for interpolating within 1D and 2D arrays.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "interpolate.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: get_float_index
 * Description: Return the interpolated index of an array based on an input.
 ********************************************************************************/
float get_float_index(float input, const float *array, int max_idx, bool is_ascending)
{
    // Handle edge cases where input value is out of bounds
    if (is_ascending)
    {
        if (input <= array[0])
        {
            return (float)0.0; // Input is less than or equal to the first element (ascending)
        }
        if (input >= array[max_idx - 1])
        {
            return static_cast<float>(max_idx - 1); // Input is greater than or equal to the last element (ascending)
        }
    }
    else
    {
        if (input >= array[0])
        {
            return (float)0.0; // Input is greater than or equal to the first element (descending)
        }
        if (input <= array[max_idx - 1])
        {
            return static_cast<float>(max_idx - 1); // Input is less than or equal to the last element (descending)
        }
    }

    // Find the two indices between which the input value falls
    for (int i = 0; i < max_idx - 1; i++)
    {
        // Ascending order: check if input is between array[i] and array[i + 1]
        if (is_ascending && input >= array[i] && input < array[i + 1])
        {
            float fraction = (input - array[i]) / (array[i + 1] - array[i]);
            return static_cast<float>(i) + fraction;
        }
        // Descending order: check if input is between array[i] and array[i + 1]
        if (!is_ascending && input <= array[i] && input > array[i + 1])
        {
            float fraction = (input - array[i + 1]) / (array[i] - array[i + 1]);
            return static_cast<float>(i) + fraction;
        }
    }

    return -1.0f; // Return -1.0f if no valid index is found (shouldn't happen with valid inputs)
}

/********************************************************************************
 * Function: get_interpolated_value
 * Description: Return the interpolated value of an array given a floating
 *              point index.
 ********************************************************************************/
float get_interpolated_value(float idx, const float *array, int max_idx)
{
    // Get the integer part of the index and the fractional part
    int lower_idx = static_cast<int>(idx);
    float fraction = idx - lower_idx;

    // Handle the case where the index is at the bounds of the array
    if (lower_idx >= max_idx - 1)
    {
        return array[max_idx - 1];
    }

    // If the index is exactly at an integer, return the corresponding array value
    if (fraction == 0.0f)
    {
        return array[lower_idx];
    }

    // Interpolate between the lower index and the next index
    float interp_value = array[lower_idx] + (array[lower_idx + 1] - array[lower_idx]) * fraction;

    return interp_value;
}

/********************************************************************************
 * Function: bilinear_interpolate_2d
 * Description: Return an interpolated value from a 2D array using bilinear
 *              interpolation. Accepts two floating point indices, one for the
 *              row and one for the column.
 ********************************************************************************/
float get_2d_interpolated_value(const float *array, int max_rows, int max_cols, float row_idx, float col_idx)
{
    // Clamp col_idx and row_idx to be within valid bounds
    if (col_idx < 0)
    {
        col_idx = 0;
    }
    if (col_idx >= max_cols)
    {
        col_idx = max_cols - 1;
    }
    if (row_idx < 0)
    {
        row_idx = 0;
    }
    if (row_idx >= max_rows)
    {
        row_idx = max_rows - 1;
    }

    // Get the integer parts of the indices
    int x0 = static_cast<int>(std::floor(col_idx));
    int y0 = static_cast<int>(std::floor(row_idx));

    // Clamp x1 and y1 to be within array bounds
    int x1 = (x0 + 1 < max_cols) ? x0 + 1 : x0;
    int y1 = (y0 + 1 < max_rows) ? y0 + 1 : y0;

    // Get the fractional parts of the indices
    float x_frac = col_idx - x0;
    float y_frac = row_idx - y0;

    // Get the values at the four corners
    float v00 = array[y0 * max_cols + x0]; // Accessing array in row-major order
    float v01 = array[y0 * max_cols + x1];
    float v10 = array[y1 * max_cols + x0];
    float v11 = array[y1 * max_cols + x1];

    // Perform bilinear interpolation
    float v0 = (1 - x_frac) * v00 + x_frac * v01; // Interpolate in col_idx direction
    float v1 = (1 - x_frac) * v10 + x_frac * v11; // Interpolate in col_idx direction

    float v = (1 - y_frac) * v0 + y_frac * v1; // Interpolate in row_idx direction

    return v;
}

/********************************************************************************
 * Function: find_floor_index
 * Description: Perform a binary search to find the index that most closely
 *              matches an input to an array.
 ********************************************************************************/
int find_floor_index(float x, float *arr, int arr_len)
{
    int idx = 0;
    int max_idx = arr_len - 1;

    // Handle cases where x is out of bounds (too small or too large)
    if (x < arr[idx])
    {
        return idx;
    }

    if (x >= arr[max_idx])
    {
        return max_idx;
    }

    // Perform binary search to find the floor index
    while (idx < max_idx)
    {
        // Round up to avoid infinite loop
        int mid = (idx + max_idx + 1) / 2;

        if (arr[mid] <= x)
        {
            // If mid is <= x, it's a potential candidate for the floor
            idx = mid;
        }
        else
        {
            // Mid is too large, so move the max_idx pointer down
            max_idx = mid - 1;
        }
    }

    return idx;
}
