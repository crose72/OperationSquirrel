#include <iostream>
#include <signal.h>
#include <time.h>
#include <chrono>

bool stopProgram = false;

// Function to handle timer interrupt for 1Hz
void timerHandler1(int sig, siginfo_t* si, void* uc)
{
    std::cout << "Timer 1: Executing function at 1Hz\n";
}

// Function to handle timer interrupt for 2Hz
void timerHandler2(int sig, siginfo_t* si, void* uc)
{
    std::cout << "Timer 2: Executing function at 2Hz\n";
}

// Function to handle timer interrupt for 10Hz
void timerHandler10(int sig, siginfo_t* si, void* uc)
{
    std::cout << "Timer 10: Executing function at 10Hz\n";
}

// Function to handle signal to stop the program
void stopHandler(int sig)
{
    stopProgram = true;
}

// Function to get current time in milliseconds
long long getCurrentTimeMs()
{
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

int main()
{
    struct sigevent sev[3];
    struct itimerspec its[3];
    timer_t timerID[3];

    // Set up the signal handler for stopping the program
    struct sigaction sa_stop;
    sa_stop.sa_handler = stopHandler;
    sigemptyset(&sa_stop.sa_mask);
    sa_stop.sa_flags = 0;
    sigaction(SIGINT, &sa_stop, NULL); // Register SIGINT (Ctrl+C) handler

    // Set up the signal handlers for the timers
    struct sigaction sa1, sa2, sa10;
    sa1.sa_flags = SA_SIGINFO;
    sa1.sa_sigaction = timerHandler1;
    sigemptyset(&sa1.sa_mask);
    sigaction(SIGRTMIN, &sa1, NULL);

    sa2.sa_flags = SA_SIGINFO;
    sa2.sa_sigaction = timerHandler2;
    sigemptyset(&sa2.sa_mask);
    sigaction(SIGRTMIN + 1, &sa2, NULL);

    sa10.sa_flags = SA_SIGINFO;
    sa10.sa_sigaction = timerHandler10;
    sigemptyset(&sa10.sa_mask);
    sigaction(SIGRTMIN + 2, &sa10, NULL);

    // Create and set up the timers
    int frequencies[3] = {1, 2, 10}; // 1Hz, 2Hz, and 10Hz
    for (int i = 0; i < 3; i++)
    {
        sev[i].sigev_notify = SIGEV_SIGNAL;
        sev[i].sigev_signo = SIGRTMIN + i;
        sev[i].sigev_value.sival_ptr = &its[i].it_value;

        timer_create(CLOCK_REALTIME, &sev[i], &timerID[i]);

        its[i].it_interval.tv_sec = 0;
        its[i].it_interval.tv_nsec = 1000000000 / frequencies[i]; // nanoseconds per period
        its[i].it_value = its[i].it_interval;
    }

    // Start the timers
    for (int i = 0; i < 3; i++)
    {
        timer_settime(timerID[i], 0, &its[i], NULL);
    }

    // Get the start time
    long long startTimeMs = getCurrentTimeMs();

    // Run the program until the stop condition is true
    while (!stopProgram)
    {
        // Your main program logic here
        // ...

        // Calculate elapsed time since code execution
        long long currentTimeMs = getCurrentTimeMs();
        long long elapsedTimeMs = currentTimeMs - startTimeMs;

        // Print the elapsed time
        std::cout << "Elapsed Time: " << elapsedTimeMs << " ms\n";

        // Sleep briefly to allow timers and interrupts to execute
        //usleep(1000); // 1 millisecond (adjust the duration as needed)
    }

    // Delete the timers
    for (int i = 0; i < 3; i++)
    {
        timer_delete(timerID[i]);
    }

    return 0;
}
