#include <hidef.h>
#include "derivative.h"
#include "iic.h"

float median(int *array, int length);


unsigned int overflow_count; // Count the number of times the timer overflows during a pulse  width measurement
 


/*! Performs repeated input-capture operations on the LIDAR PWM output for a specified number of samples,
and returns the median sample value. */
float get_distance(int samples) {

    float distance_array[20];   // Array to store the samples

    int i;
    
    float distance;  // Distance in mm
    
    unsigned int edge1, edge2;
    
    segment_stop();   // Disable 7-seg interrupts
    
    for (i=0; i<samples; i++) {
    
        unsigned long int pulse_width = 0;
        
        DisableInterrupts;
        overflow_count = 0;                  // Reset overflow count   

        TSCR1 = 0x80;                        // Timer enabled 
        TSCR2 = 0x00;                        // Prescalar set to 1
        TIOS &= ~TIOS_IOS1_MASK;             // Input capture enabled on Pin 1 
        TCTL4 = 0x04;                        // Set Timer Control Register 4 so that it captures input on rising edges only
        TFLG1 = TFLG1_C1F_MASK;              // Clear timer interrupt flag for channel 1


        while(!(TFLG1 & TFLG1_C1F_MASK));    // Wait for first rising edge
        TFLG1 = TFLG1_C1F_MASK;              // Clear timer interrupt flag for channel 1

        TFLG2 = 0x80;                        // Clear timer reset at overflow flag 
        TSCR2 |= 0x80;                       // Enable the timer overflow interrupt

        EnableInterrupts;

        edge1 = TC1;                         // Store the timer count on the rising edge
        TCTL4 = 0x08;                        // Reset the Timer CR 4 to capture falling edges only 

        while(!(TFLG1 & TFLG1_C1F_MASK));    // Wait for next consecutive falling edge
        edge2 = TC1;                         // Store the timer count on the falling edge
        
        pulse_width = edge2 - edge1;         // Calculate the pulse width by subtracting rising and falling edge

        if(edge2 < edge1){                   // If edge1 is greater than edge2  
            overflow_count -= 1;          
        }
        
        pulse_width += (unsigned long)overflow_count * 65536;  // Account for timer overflow  
        distance_array[i] = (float)(pulse_width)/24;
    }
    
    distance = median(distance_array, samples);    // Median of the samples
        
    if (distance > 30) {
        distance -= 30;       // Calibration value
    }
    
    segment_init();   // Enable 7-seg interrupts
        
    return distance;

}     
 
/*! Returns the median value of a numerical array. */
float median(float *array, int length) {

    float temp;
    int i, j;

   // Sort the array in ascending order
    for(i = 0; i < length-1; i++) {
        for(j=i + 1; j < length; j++) {
            if(array[j] < array[i]) {
                temp = array[i];
                array[i] = array[j];
                array[j] = temp;
            }
        }
    }

    // If array is even length, return the average of the middle 2 numbers
    // If array is odd length, return the middle number
    if (length%2 == 0) {
        return((array[length/2] + array[length/2 - 1]) / 2.0);
    } else {
        return array[length/2];
    }
}



/*** Timer Overflow Interrrupt ***/
/*! This interrupt is used by the get_distance() function to count the number of timer overflows during the input-capture process. */
interrupt 16 void timov(void){
    TFLG2 |= 0x80;           // Set bit 7 to clear the flag  
    overflow_count++;        // Increment the overflow count
}


  