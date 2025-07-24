/********************************************************************************
 * @file    vector_math.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Utilities for vector operations.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "vector_math.h"

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
 * Function: vector_sum
 * Description: Return the sum of all elements of a vector
 ********************************************************************************/
float vector_sum(const std::vector<float> &x)
{
    float sum = 0.0;

    for (int i = 0; i < x.size(); ++i)
    {
        sum += x[i];
    }

    return sum;
}

float vector_product_sum(const std::vector<float> &x, const std::vector<float> &y)
{
    float sum = 0.0;

    for (int i = 0; i < x.size(); ++i)
    {
        sum += (x[i] * y[i]);
    }

    return sum;
}

float vector_square_sum(const std::vector<float> &x)
{
    float sum = 0.0;

    for (int i = 0; i < x.size(); ++i)
    {
        sum += (x[i] * x[i]);
    }

    return sum;
}

float vector_diff_square_sum(const std::vector<float> &x, const std::vector<float> &y)
{
    float sum = 0.0;

    for (int i = 0; i < x.size(); ++i)
    {
        sum += std::pow((x[i] - y[i]), 2);
    }

    return sum;
}

float vector_stdev(const std::vector<float> &x)
{
    int n = x.size();
    float avg = vector_sum(x) / n;
    float variance = 0.0;

    for (int i = 0; i < n; ++i)
    {
        variance += std::pow((x[i] - avg), 2);
    }

    variance /= n;

    return std::sqrt(variance);
}