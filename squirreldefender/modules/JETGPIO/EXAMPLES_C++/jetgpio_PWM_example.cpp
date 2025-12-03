/* Usage example of the JETGPIO library
 * Compile with: g++ -Wall -o jetgpio_PWM_example jetgpio_PWM_example.cpp -ljetgpio
 * Execute with: sudo ./jetgpio_PWM_example
 */

#include <iostream>
#include <unistd.h>
#include <jetgpio.h>

int main(int argc, char *argv[])
{
  int Init;

  Init = gpioInitialise();
  if (Init < 0)
    {
      /* jetgpio initialisation failed */
      printf("Jetgpio initialisation failed. Error code:  %d\n", Init);
      exit(Init);
    }
  else
    {
      /* jetgpio initialised okay*/
      printf("Jetgpio initialisation OK. Return code:  %d\n", Init);
    }	

/* Setting up PWM frequency=10kHz @ pin 15 */

  int PWMstat = gpioSetPWMfrequency(15, 100000);

  if (PWMstat < 0)
    {
      /* PWM frequency set up failed */
      printf("PWM frequency set up failed. Error code:  %d\n", PWMstat);
      exit(Init);
    }
  else
    {
      /* PWM frequency set up okay*/
      printf("PWM frequency set up okay at pin 15. Return code:  %d\n", PWMstat);
    }
  /* Set up PWM duty cycle to approx 50% (0=0% to 256=100%) @ pin 15*/
  int PWMstat2 = gpioPWM(15, 256);

  if (PWMstat2 < 0)
    {
      /* PWM start on failed */
      printf("PWM start failed. Error code:  %d\n", PWMstat2);
      exit(Init);
    }
  else
    {
      /* PWM started on okay*/
      printf("PWM started up okay at pin 15. Return code:  %d\n", PWMstat2);
    }

  int x =0;
  printf("PWM going at pin 15 for 4 seconds\n");
  while (x<2) {
    sleep(2);
    x++;
  }
  // Terminating library 
  gpioTerminate();
  printf("PWM stopped, bye!\n");
  exit(0);
	
}

