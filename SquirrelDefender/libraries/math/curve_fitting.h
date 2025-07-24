#pragma once

/********************************************************************************
 * @file    curve_fitting.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef CURVE_FITTING_H
#define CURVE_FITTING_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <stdint.h>
#include <cmath>
#include "vector_math.h"
#include <vector>

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
void linear_regression_window(const std::vector<float> &x, const std::vector<float> &y, float *slope, float *intercept);
float calc_rmse(const std ::vector<float> &x, const std ::vector<float> &y, float slope, float intercept);
void count_points_above_and_below(const std ::vector<float> &x, const std ::vector<float> &y, float slope, float intercept, int *above, int *below);

#endif // CURVE_FITTING_H
