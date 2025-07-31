#pragma once

/********************************************************************************
 * @file    signal_processing.h
 * @author  Cameron Rose
 * @date    4/29/2025
 ********************************************************************************/
#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <stdint.h>
#include <cmath>
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
float low_pass_filter(float x, float x_prev, float alpha);
void update_buffer(std::vector<float> &buffer, float sample, int &index);
float moving_average(std::vector<float> &buffer, float sample, int &index, float &sum);
std::vector<float> unwrap_buffer(const std::vector<float> &buffer, int index);
float first_derivative(float current, float previous, float dt);
float second_derivative(float current, float prev, float prev2, float dt);

#endif // SIGNAL_PROCESSING_H
