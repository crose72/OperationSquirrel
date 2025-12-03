/* Usage example of the JETGPIO library
 * Compile with: g++ -Wall -o jetgpio_PWM_servo_example jetgpio_PWM_servo_example.cpp -ljetgpio
 * Execute with: sudo ./jetgpio_PWM_example
 */

#include <iostream>
#include <unistd.h>
#include <jetgpio.h>

int main()
{
    int Init = gpioInitialise();
    if (Init < 0) {
        printf("Jetgpio initialisation failed. Error code:  %d\n", Init);
        return Init;
    }

    printf("Jetgpio initialisation OK. Return code:  %d\n", Init);

    // Set PWM frequency to 50Hz (standard for servo control)
    int PWMstat = gpioSetPWMfrequency(15, 50);
    if (PWMstat < 0) {
        printf("PWM frequency set up failed. Error code:  %d\n", PWMstat);
        return Init;
    }

    printf("PWM frequency set to 50Hz at pin 15.\n");

    // Sweep servo from 0° to 180° and back
    for (int angle = 0; angle <= 180; angle += 30) {
        // Convert angle to PWM duty cycle (range 0-256)
        // 1ms → 5%, 2ms → 10% duty cycle of 20ms period
        // So: duty = map(angle, 0, 180, 13, 26)
        int duty = 13 + (angle * (26 - 13)) / 180;
        gpioPWM(15, duty);
        printf("Set angle %d -> duty %d\n", angle, duty);
        sleep(1);
    }

    // Hold at midpoint for 2 sec
    gpioPWM(15, 19);  // ~1.5ms pulse
    sleep(2);

    gpioTerminate();
    printf("Servo test done.\n");

    return 0;
}
