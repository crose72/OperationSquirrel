#include "time_calc.h"

float elapsedTimeSeconds = (float)0.0;
const float timeStep = (float)0.025;
std::chrono::time_point<std::chrono::high_resolution_clock> startTime = std::chrono::high_resolution_clock::now();
std::chrono::time_point<std::chrono::high_resolution_clock> currentTime = std::chrono::high_resolution_clock::now();
std::chrono::duration<float, std::milli> elapsedTime(0.0);

void calcStartTimeMS(void)
{
    // Get the start time
    startTime = std::chrono::high_resolution_clock::now();
}

void calcExecutionTime(void)
{
    if (firstLoopAfterStartup == true)
    {
        elapsedTimeSeconds = (float)0.0;
    }
    else
    {
        // Get the current timestamp
        currentTime = std::chrono::high_resolution_clock::now();

        // Calculate the elapsed time since the start of the program
        elapsedTime = currentTime - startTime;

        // Convert elapsed time to seconds with millisecond precision
        float elapsedTimeSecondsTMP = elapsedTime.count() / 1000.0;

        // Truncate the number to three decimal places
        elapsedTimeSeconds = (floor(elapsedTimeSecondsTMP * 1000.0) / 1000.0 - timeStep);

        // Print program run time
        // printf("Elapsed Time: %0.3f\n", elapsedTimeSeconds);
    }
}