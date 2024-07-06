#include <iostream>
#include <JetsonGPIO.h>
#include <csignal>
#include <thread>

bool stop_program = false;
bool save_button_press = false;
bool save_button_press_prv = false;
int button_state_prv = GPIO::LOW;
const int SAVE_BUTTON_PIN1 = 21;
const int SAVE_BUTTON_PIN2 = 22;

void sig_handler(int signo)
{
    if (signo == SIGINT)
    {
        stop_program = true;
        std::cout << "received SIGINT\n"
                  << std::endl;
    }
}

void attach_sig_handler(void)
{
    if (signal(SIGINT, sig_handler) == SIG_ERR)
    {
        std::cout << "can't catch SIGINT" << std::endl;
    }
}

void save_video_button_state(void)
{
    unsigned int button_state;
    button_state = GPIO::input(SAVE_BUTTON_PIN2);

    if (button_state_prv == GPIO::LOW && button_state == GPIO::HIGH)
    {
        std::cout << "I'm free" << std::endl;
        save_button_press = true;
    }
    else
    {
        std::cout << "Trapped" << std::endl;
        save_button_press = false;
    }

    button_state_prv = button_state;
    save_button_press_prv = save_button_press;
}

bool gpio_init(void)
{
    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(SAVE_BUTTON_PIN2, GPIO::IN); // starts low, matches SAVE_BUTTON_PIN1

    return true;
}

int main()
{
    attach_sig_handler();
    gpio_init();

    while (!stop_program)
    {
        save_video_button_state();
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Add a delay to debounce the button and reduce CPU usage
    }

    GPIO::cleanup();

    return 0;
}