#include <iostream>
#include <signal.h>
#include <chrono>
#include <time.h>
#include <mutex>

bool stopProgram = false;
std::mutex mutex;
long long startTimeMs; // Variable to store the start time in milliseconds


// Function to handle timer interrupt at 40Hz
void timerHandler(int sig, siginfo_t* si, void* uc)
{
    std::lock_guard<std::mutex> lock(mutex);

    // Get the current timestamp in milliseconds
    auto now = std::chrono::high_resolution_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    // Calculate the elapsed time since the start of the program
    long long elapsedTimeMs = timestamp - startTimeMs;

    // Print the elapsed time and the function name
    std::cout << "Elapsed Time: " << elapsedTimeMs << "\n";
}

// Function to handle signal to stop the program
void stopHandler(int sig)
{
    stopProgram = true;
}

int main()
{
    // Set up the signal handler for stopping the program
    struct sigaction sa_stop;
    sa_stop.sa_handler = stopHandler;
    sigemptyset(&sa_stop.sa_mask);
    sa_stop.sa_flags = 0;
    sigaction(SIGINT, &sa_stop, nullptr); // Register SIGINT (Ctrl+C) handler

    // Set up the signal handler for the timer
    struct sigaction sa_timer;
    sa_timer.sa_flags = SA_SIGINFO;
    sa_timer.sa_sigaction = timerHandler;
    sigemptyset(&sa_timer.sa_mask);
    sigaction(SIGRTMIN, &sa_timer, nullptr);

    // Create and set up the timer
    timer_t timerID;
    struct sigevent sev;
    struct itimerspec its;

    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGRTMIN;
    sev.sigev_value.sival_ptr = &timerID;

    timer_create(CLOCK_REALTIME, &sev, &timerID);

    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 1000000000 / 40; // nanoseconds per period for 40Hz
    its.it_value = its.it_interval;


    // Get the start time
    auto now = std::chrono::high_resolution_clock::now();
    startTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    timer_settime(timerID, 0, &its, nullptr);

    // Run the program until the stop condition is true
    while (!stopProgram)
    {
        // Lock the mutex to prevent concurrent execution
        //std::lock_guard<std::mutex> lock(mutex);

        // Your main program logic here
        // ...
    }

    // Delete the timer
    timer_delete(timerID);

    return 0;
}
