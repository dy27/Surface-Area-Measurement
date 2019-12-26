#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */

#include "delay.h"        // Assembly function for delays


/*! Initialise the PWM module settings for the pan and tilt servo motors. */
void servo_init(void) {

    PWMCLK = 0b00000000;          // Use clocks A and B
    PWMPOL = 0b10100000;          // PWM signal starts HIGH
    PWMPRCLK = 0b01000100;        // Prescale clocks A and B by 16    
    PWMCAE = 0b00000000;          // Left align PWM waveform
    PWMCTL = 0b11000000;          // Concatenate PWM channels 4 and 5, 6 and 7 
    
    PWMPER4 = 0x75;               // Set period to 30000
    PWMPER5 = 0x30;
    
    PWMPER6 = 0x75;               // Set period to 30000
    PWMPER7 = 0x30;

    PWME = 0b11110000;            // Enable PWM channels 4,5,6 and 7.
}

/*! Move the tilt servo to the specified angle. */
void tilt_servo(float angle) {

    int duty = (int)(2350 + 15.85*(angle-90));  // Formula after calibration
    
    //int duty = (int) 2250 + 12*(angle-90); // Theoretical formula
    
    PWMDTY4 = duty >> 8;       // Set duty cycle
    PWMDTY5 = duty & 0x00FF;   //
  
}

/*! Move the pan servo to the specified angle. */
void pan_servo(float angle) {

    int duty = (int)(2350 + 15.85*(angle-90));
    
    PWMDTY6 = duty >> 8;       // Set duty cycle
    PWMDTY7 = duty & 0x00FF;   //

}