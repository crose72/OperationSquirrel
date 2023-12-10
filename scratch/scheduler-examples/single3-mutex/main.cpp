#include <mutex>
#include <signal.h>
#include <chrono>

void initialize(void);
void task_25ms(int sig, siginfo_t* si, void* uc);
void stopHandler(int sig);
void startTask_25ms(void);
void stopTask_25ms(void);
void setupTask_25ms(void);

std::mutex mutex;
timer_t timerID;
struct sigaction sa;
struct sigevent sev;
struct itimerspec its;
bool stopProgram = false;

void setupTask_25ms(void)
{
    // Set up the signal handler for stopping the program
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = task_25ms;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGRTMIN, &sa, nullptr);

    // Set up the signal handler for the timer
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = task_25ms;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGRTMIN, &sa, nullptr);

    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGRTMIN;
    sev.sigev_value.sival_ptr = &timerID;

    timer_create(CLOCK_REALTIME, &sev, &timerID);

    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 25e6; // nanoseconds per period for 40Hz
    its.it_value = its.it_interval;
}

void startTask_25ms(void)
{
    timer_settime(timerID, 0, &its, nullptr);
}

void stopTask_25ms(void)
{
    timer_delete(timerID);
}

void stopHandler(int sig)
{
    stopProgram = true;
}

void initialize(void)
{
    printf("Initialize\n");
}

// Function to handle timer interrupt at 40Hz
void task_25ms(int sig, siginfo_t* si, void* uc)
{
    std::lock_guard<std::mutex> lock(mutex);
    printf("25ms function call\n");
}

int main() 
{
    initialize(); // stuff that runs once at the beginning
    startTask_25ms(); // periodic task initiated at the start of main()

    while (!stopProgram) 
	{
        // Main code is executed in task_25ms()
    }

    stopTask_25ms();

    return 0;
}

