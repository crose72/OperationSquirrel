/********************************************************************************
 * @file    curve_fitting.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Utilities for regression
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "curve_fitting.h"

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
 * Function: linear_regression_window
 * Description: Linear curve fit of a history of x and y points.
 ********************************************************************************/
void linear_regression_window(const std::vector<float> &x, const std::vector<float> &y, float *slope, float *intercept)
{
    int n = x.size();
    int m = y.size();

    // vectors must be same size
    if (n == m)
    {
        float x_sum = (float)0.0;
        float y_sum = (float)0.0;
        float x_prod_y_sum = (float)0.0;
        float x_square_sum = (float)0.0;

        x_sum = vector_sum(x);
        y_sum = vector_sum(y);
        x_prod_y_sum = vector_product_sum(x, y);
        x_square_sum = vector_square_sum(x);

        *slope = (n * x_prod_y_sum - x_sum * y_sum) / (n * x_square_sum - x_sum * x_sum);
        *intercept = (y_sum - *slope * x_sum) / n;
    }
    else
    {
        *slope = (float)0.0;
        *intercept = (float)0.0;
    }
}

/********************************************************************************
 * Function: calc_rmse
 * Description: Analysis of the fitness of a curve fit.
 ********************************************************************************/
float calc_rmse(const std ::vector<float> &x, const std ::vector<float> &y, float slope, float intercept)
{
    float rmse = 0.0;
    float sum_diff_square = 0.0;
    int n = x.size();

    for (int i = 0; i < n; ++i)
    {
        float y_f = (slope * x[i] + intercept);
        float diff_square = std::pow((y[i] - y_f), 2);
        sum_diff_square += diff_square;
    }

    return std::sqrt(sum_diff_square / n);
}

/********************************************************************************
 * Function: count_points_above_and_below
 * Description: Count the number of points above and below a linear curve fit.
 ********************************************************************************/
void count_points_above_and_below(const std ::vector<float> &x, const std ::vector<float> &y, float slope, float intercept, int *above, int *below)
{
    int points_above = 0;
    int points_below = 0;
    int n = x.size();

    for (int i = 0; i < n; ++i)
    {
        float y_f = (slope * x[i] + intercept);
        float diff = y[i] - y_f;

        if (diff > (float)0.0)
        {
            ++points_above;
        }
        else if (diff < (float)0.0)
        {
            ++points_below;
        }
    }

    *above = points_above;
    *below = points_below;
}