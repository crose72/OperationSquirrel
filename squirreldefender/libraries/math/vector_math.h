/********************************************************************************
 * @file    vector_math.h
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Utilities for vector operations.
 ********************************************************************************/
#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <vector>

/********************************************************************************
 * Typedefs / Enums / Structs
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
float vector_sum(const std::vector<float> &x);
float vector_product_sum(const std::vector<float> &x, const std::vector<float> &y);
float vector_square_sum(const std::vector<float> &x);
float vector_diff_square_sum(const std::vector<float> &x, const std::vector<float> &y);
float vector_stdev(const std::vector<float> &x);

#endif // VECTOR_MATH_H
