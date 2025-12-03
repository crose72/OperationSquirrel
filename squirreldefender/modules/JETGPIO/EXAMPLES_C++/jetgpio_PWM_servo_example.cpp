/* Servo control example using JetGPIO's gpioPWM()
 * Compile: g++ -Wall -o jetgpio_PWM_servo_example jetgpio_PWM_servo_example.cpp -ljetgpio
 * Run with: sudo ./jetgpio_PWM_servo_example
 */

#include <iostream>
#include <unistd.h>
#include <jetgpio.h>

// === New Helpers for Servo Control ===

int mapAngleToMicroseconds(int angle) {
    if (angle < 0) angle = 0;
    else if (angle > 180) angle = 180;
    return 1000 + (angle * 1000) / 180;  // 1000–2000 µs
}

int microsecondsToDutyCycle(int pulse_width_us) {
    return (pulse_width_us * 256) / 20000;  // Scale to 0–256 range for 20ms period
}

int gpioServoWrite(unsigned gpio, int angle) {
    int pulse_us = mapAngleToMicroseconds(angle);
    int duty = microsecondsToDutyCycle(pulse_us);
    return gpioPWM(gpio, duty);
}

// === New Helpers for Servo Control ===

const int pwmPin = 15;

int main() {
    // Initialize JetGPIO
    int status = gpioInitialise();
    if (status < 0) {
        std::cerr << "JetGPIO initialization failed. Error: " << status << std::endl;
        return 1;
    }
    std::cout << "JetGPIO initialized.\n";

    gpioSetPWMfrequency(15, 50);  // Configure GPIO 15 for 50Hz PWM
    gpioServoWrite(15, 0);        // Full left
    sleep(1);
    gpioServoWrite(15, 90);       // Middle
    sleep(1);
    gpioServoWrite(15, 180);      // Full right
    sleep(1);
    return 0;

    // Turn off PWM (set duty cycle to 0)
    gpioPWM(pwmPin, 0);
    gpioTerminate();
    std::cout << "Servo test done.\n";
}
