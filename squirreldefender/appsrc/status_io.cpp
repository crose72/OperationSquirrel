#ifdef BLD_JETSON_B01

/********************************************************************************
 * @file    status_io.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Handles GPIO status indicators and button input on the Jetson Nano
 *          B01. Provides LED feedback sequences for initialization, runtime
 *          status, error states, and program completion, as well as monitoring
 *          the SAVE button input. Designed for the embedded-style init/loop/
 *          shutdown architecture.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "status_io.h"
#include <JetsonGPIO.h>
#include <thread>
#include <chrono>
#include <unistd.h>

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
bool g_save_button_press;
int button_state_prv;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
const int red_led_pin = 18;
const int green_led_pin = 19;
const int save_button_pin1 = 21; // pin1 kept for legacy wiring, unused by current code
const int save_button_pin2 = 22;

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: StatusIO
 * Description: Constructor
 ********************************************************************************/
StatusIO::StatusIO(void) {}

/********************************************************************************
 * Function: StatusIO
 * Description: Destructor
 ********************************************************************************/
StatusIO::~StatusIO(void) {}

/********************************************************************************
 * Function: status_initializing
 * Description: Use this sequence to indicate program is still in the init phase.
 ********************************************************************************/
void StatusIO::status_initializing(void)
{
    for (int j = 0; j < 3; j++)
    {
        // Blink three times
        for (int i = 0; i < 3; i++)
        {
            GPIO::output(red_led_pin, GPIO::HIGH);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(red_led_pin, GPIO::LOW);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(green_led_pin, GPIO::HIGH);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(green_led_pin, GPIO::LOW);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        // Pause before repeating
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    GPIO::output(green_led_pin, GPIO::HIGH);
    GPIO::output(red_led_pin, GPIO::HIGH);
}

/********************************************************************************
 * Function: status_good
 * Description: Indicate that the system is good with a green led.
 ********************************************************************************/
void StatusIO::status_good(void)
{
    GPIO::output(green_led_pin, GPIO::HIGH);
    GPIO::output(red_led_pin, GPIO::LOW);
}

/********************************************************************************
 * Function: status_bad
 * Description: Indicate that the system is bad with a red led.
 ********************************************************************************/
void StatusIO::status_bad(void)
{
    GPIO::output(red_led_pin, GPIO::HIGH);
    GPIO::output(green_led_pin, GPIO::LOW);
}

/********************************************************************************
 * Function: status_bad_blink
 * Description: Indicate that the system is bad with a red led.
 ********************************************************************************/
void StatusIO::status_bad_blink(void)
{
    GPIO::output(green_led_pin, GPIO::LOW);

    for (int i = 0; i < 3; i++)
    {
        GPIO::output(red_led_pin, GPIO::HIGH);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(red_led_pin, GPIO::LOW);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(red_led_pin, GPIO::HIGH);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(red_led_pin, GPIO::LOW);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(red_led_pin, GPIO::HIGH);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(red_led_pin, GPIO::LOW);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(red_led_pin, GPIO::HIGH);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    GPIO::output(red_led_pin, GPIO::HIGH);
}

/********************************************************************************
 * Function: clear_all_leds
 * Description: All leds are turned off.
 ********************************************************************************/
void StatusIO::clear_all_leds(void)
{
    GPIO::output(green_led_pin, GPIO::LOW);
    GPIO::output(red_led_pin, GPIO::LOW);
}

/********************************************************************************
 * Function: save_video_button_state
 * Description: This function will monitor the state of the button.
 ********************************************************************************/
void StatusIO::save_video_button_state(void)
{
    unsigned int button_state;
    button_state = GPIO::input(save_button_pin2);

    if (button_state_prv == GPIO::LOW && button_state == GPIO::HIGH)
    {
        g_save_button_press = true;
    }
    else
    {
        g_save_button_press = false;
    }

    button_state_prv = button_state;
}

/********************************************************************************
 * Function: status_program_complete
 * Description: Use this sequence to indicate program has completed.
 ********************************************************************************/
void StatusIO::status_program_complete(void)
{
    for (int j = 0; j < 3; j++)
    {
        // Blink three times
        for (int i = 0; i < 3; i++)
        {
            GPIO::output(green_led_pin, GPIO::HIGH);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(green_led_pin, GPIO::LOW);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(green_led_pin, GPIO::HIGH);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(green_led_pin, GPIO::LOW);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(green_led_pin, GPIO::HIGH);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(green_led_pin, GPIO::LOW);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        // Pause before repeating
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    GPIO::output(green_led_pin, GPIO::HIGH);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
}

/********************************************************************************
 * Function: init
 * Description: Initialize the pins on the jetson.
 ********************************************************************************/
bool StatusIO::init(void)
{
    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(green_led_pin, GPIO::OUT, GPIO::LOW);
    GPIO::setup(red_led_pin, GPIO::OUT, GPIO::LOW);
    GPIO::setup(save_button_pin2, GPIO::IN); // starts low, matches save_button_pin1
    g_save_button_press = false;
    button_state_prv = GPIO::LOW;

    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Loop for all IO that needs to be monitored continuously.
 ********************************************************************************/
void StatusIO::loop(void)
{
    save_video_button_state();
}

/********************************************************************************
 * Function: shutdown
 * Description: Cleanup tasks for jetson IO.
 ********************************************************************************/
void StatusIO::shutdown(void)
{
    GPIO::cleanup();
}

#endif // BLD_JETSON_B01
