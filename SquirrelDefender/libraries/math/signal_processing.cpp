/********************************************************************************
 * @file    signal_processing.cpp
 * @author  Cameron Rose
 * @date    4/29/2025
 * @brief   Utilities for signal processing, such as low/high pass filters.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "signal_processing.h"

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
 * Function: low_pass_filter
 * Description: Return the filtered value of an input x with filter coeff alpha.
 ********************************************************************************/
float low_pass_filter(float x, float x_prev, float alpha)
{
    return ((float)1.0 - alpha) * x + alpha * x_prev;
}

void update_buffer(std::vector<float> &buffer, float sample, int &index)
{
    int window_size = buffer.size();

    buffer[index] = sample;

    // Move the index - resets to 0 when window size is reached
    if ((index + 1) == window_size)
    {
        index = (int)0;
    }
    else
    {
        ++index;
    }
}

std::vector<float> unwrap_buffer(const std::vector<float> &buffer, int index)
{
    std::vector<float> linear_buffer;
    int n = buffer.size();

    linear_buffer.reserve(n);

    for (int i = 0; i < n; ++i)
    {
        linear_buffer.push_back(buffer[(index + i) % n]);
    }

    return linear_buffer;
}

float moving_average(std::vector<float> &buffer, float sample, int &index, float &sum)
{
    int window_size = buffer.size();

    // Subtract old value from the running sum
    sum -= buffer[index];

    // Place sample in current index
    buffer[index] = sample;

    // Add new sample to the running sum
    sum += sample;

    // Move the index - resets to 0 when window size is reached
    if ((index + 1) == window_size)
    {
        index = (int)0;
    }
    else
    {
        ++index;
    }

    return sum / window_size;
}

float first_derivative(float current, float previous, float dt)
{
    return (current - previous) / dt;
}

float second_derivative(float current, float prev, float prev2, float dt)
{
    return (current - 2 * prev + prev2) / (dt * dt);
}