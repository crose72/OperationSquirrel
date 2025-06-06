#ifdef BLD_JETSON_B01

/********************************************************************************
 * @file    status_io.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Configure Jetson GPIO and provide methods for using the pins on the
 *          Jetson B0!.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "status_io.h"

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
const int RED_LED_PIN = 18;
const int GREEN_LED_PIN = 19;
const int SAVE_BUTTON_PIN1 = 21;
const int SAVE_BUTTON_PIN2 = 22;

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: StatusIO
 * Description: Constructor
 ********************************************************************************/
StatusIO::StatusIO(void) {}

/********************************************************************************
 * Function: Video
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
            GPIO::output(RED_LED_PIN, GPIO::HIGH);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(RED_LED_PIN, GPIO::LOW);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(GREEN_LED_PIN, GPIO::HIGH);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(GREEN_LED_PIN, GPIO::LOW);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        // Pause before repeating
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    GPIO::output(GREEN_LED_PIN, GPIO::HIGH);
    GPIO::output(RED_LED_PIN, GPIO::HIGH);
}

/********************************************************************************
 * Function: status_good
 * Description: Indicate that the system is good with a green led.
 ********************************************************************************/
void StatusIO::status_good(void)
{
    GPIO::output(GREEN_LED_PIN, GPIO::HIGH);
    GPIO::output(RED_LED_PIN, GPIO::LOW);
}

/********************************************************************************
 * Function: status_bad
 * Description: Indicate that the system is bad with a red led.
 ********************************************************************************/
void StatusIO::status_bad(void)
{
    GPIO::output(RED_LED_PIN, GPIO::HIGH);
    GPIO::output(GREEN_LED_PIN, GPIO::LOW);
}

/********************************************************************************
 * Function: status_bad_blink
 * Description: Indicate that the system is bad with a red led.
 ********************************************************************************/
void StatusIO::status_bad_blink(void)
{
    GPIO::output(GREEN_LED_PIN, GPIO::LOW);

    for (int i = 0; i < 3; i++)
    {
        GPIO::output(RED_LED_PIN, GPIO::HIGH);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(RED_LED_PIN, GPIO::LOW);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(RED_LED_PIN, GPIO::HIGH);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(RED_LED_PIN, GPIO::LOW);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(RED_LED_PIN, GPIO::HIGH);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(RED_LED_PIN, GPIO::LOW);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(RED_LED_PIN, GPIO::HIGH);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    GPIO::output(RED_LED_PIN, GPIO::HIGH);
}

/********************************************************************************
 * Function: clear_all_leds
 * Description: All leds are turned off.
 ********************************************************************************/
void StatusIO::clear_all_leds(void)
{
    GPIO::output(GREEN_LED_PIN, GPIO::LOW);
    GPIO::output(RED_LED_PIN, GPIO::LOW);
}

/********************************************************************************
 * Function: save_video_button_state
 * Description: This function will monitor the state of the button.
 *********************************
 ***********************************************/
void StatusIO::save_video_button_state(void)
{
    unsigned int button_state;
    button_state = GPIO::input(SAVE_BUTTON_PIN2);

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
 * Description: Use this sequence to indicate program is has completed.
 ********************************************************************************/
void StatusIO::status_program_complete(void)
{
    for (int j = 0; j < 3; j++)
    {
        // Blink three times
        for (int i = 0; i < 3; i++)
        {
            GPIO::output(GREEN_LED_PIN, GPIO::HIGH);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(GREEN_LED_PIN, GPIO::LOW);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(GREEN_LED_PIN, GPIO::HIGH);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(GREEN_LED_PIN, GPIO::LOW);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(GREEN_LED_PIN, GPIO::HIGH);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(GREEN_LED_PIN, GPIO::LOW);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        // Pause before repeating
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    GPIO::output(GREEN_LED_PIN, GPIO::HIGH);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
}

/********************************************************************************
 * Function: init
 * Description: Initialize the pins on the jetson.
 ********************************************************************************/
bool StatusIO::init(void)
{
    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(GREEN_LED_PIN, GPIO::OUT, GPIO::LOW);
    GPIO::setup(RED_LED_PIN, GPIO::OUT, GPIO::LOW);
    GPIO::setup(SAVE_BUTTON_PIN2, GPIO::IN); // starts low, matches SAVE_BUTTON_PIN1
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
 * Description: Cleanup tasks for jetson io.
 ********************************************************************************/
void StatusIO::shutdown(void)
{
    GPIO::cleanup();
}

#endif // BLD_JETSON_B01
