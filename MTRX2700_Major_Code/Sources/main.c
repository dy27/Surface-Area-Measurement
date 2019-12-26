/*****************************************************************
 *                                                               *
 * MTRX2700 Major Assignment                                     *
 *                                                               *
 *                                                               *
 * This program controls a LIDAR sensor mounted on a pan-tilt    *
 * system to measure the surface area of an object.              *
 *                                                               *
 * Refer to the documentation for a detailed description of      *
 * the system functionality.                                     *
 *                                                               *
 *                                                               *
 * Group 8                                                       *
 * James Cooper, Thejan Elankayer, Rishi Rangarajan,             *
 * Naida Rasheed, David Young                                    *
 *                                                               *
 *****************************************************************/
 

#include <hidef.h>        // Common defines and macros 
#include "derivative.h"   // Derivative-specific definitions
                                                                               
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "delay.h"        // Assembly delay functions
#include "lcd.h"          // LCD display functions
#include "servo.h"        // Servo functions
#include "lidar.h"        // LIDAR functions
#include "imu.h"          // IMU functions
#include "iic.h"          // I2C functions
#include "pll.h"
#include "sci1.h"         // Serial communication functions
#include "l3g4200.h"      // Register definitions





// *** Function Declarations ***

char getkey(void);  // Reads in a key from the keypad
char getc_buffer(void); // Reads in a character from input_buffer from PC
void getkey_string(int float_mode);  // Reads a sequence of keys from the keypad or PC

int str2num(char* string); // Converts a string to an intenger

void delay_ms(unsigned int); // Delay for a specified number of ms

void sweep(float resolution, int samples, int scan_delay); // One sweep scan, for testing

void find_edges(float resolution, float samples); // Find the approximate edges of the shape
void rectangle_scan(float resolution, int samples, int scan_delay); // Rectangle scanning algorithm
void full_scan(float resolution, int samples , int scan_delay); // Full scan algorithm

void servo_test(void);    // Servo testing mode
void lidar_test(void);    // LIDAR testing mode
void gyro_test(void);     // Gyroscope testing mode
void accel_test(void);    // Accelerometer testing mode
void magnet_test(void);   // Magnetometer testing mode

void segment_init(void);  // Begin 7-segment display interrupts
void segment_stop(void);  // Stop 7-segment display interrupts

void send_point(float pan, float tilt, float distance); // Send a data point to serial

void display_angle(float pan, float tilt);  // Display the pan and tilt angle on the LCD


// *** Global Variables ***
char keypad_buffer[10];             // Array to store keypad input
char *buffer_ptr = keypad_buffer;   // Pointer to a position in the buffer, initialised at the first element

char input_buffer[100];             // Array to store information from serial input
char *write_ptr = input_buffer;     // Pointer to indicate the next array location to store data into
char *read_ptr = input_buffer;      // Pointer to indicate the next array location to read data from

char string[20]; // String to pass to a function to store its output

float distance;    // LIDAR distance measurement

int pc_mode = 0;            // 0 for board mode, 1 for PC mode
                                                                                                                                              
int digits[16] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71};    // Look-up table for displaying digits to 7-segments
int enable_disp[4] = {0xFE, 0xFD, 0xFB, 0xF7};    // Look-up table for enabling each 7-segment separately
int segment_count = 0;      // Current 7-segment to display

char button_press;          // Variable to store the ASCII value of a button press

float shape_edges[4];       // Array to store the angles of left, right, bottom and top edges 


void main(void) {
   
    servo_init();           // Configure PWM settings 
    lcd_init();             // Initialise LCD   
    SCI1_Init(BAUD_9600);   // Initialise SCI with Baud rate 9600
    init_imu();             // Initialise IMU
      
    EnableInterrupts;
    
    

    // Get PC or board mode
        
    SCI1CR2 |= 0b00100000;    // Enable receiver interrupts
    
    cmd2LCD(0x80);        // LCD row 1                      
    putsLCD("Press key on");
    cmd2LCD(0xC0);        // LCD row 2
    putsLCD("Board or PC");
    getkey();     

    if (pc_mode == 0) {               // If PC mode has not been enabled by this point
        SCI1CR2 &= ~0b00100000;       // Disable receiver interrupts so PC mode can't be enabled mid-program             
        SCI1_OutString("b\n");        // Broadcast to PC that board mode is enabled. 
    } else {
        getc_buffer();                // If PC mode enabled, clear the character that was just entered
        getc_buffer();                // Clear newline
        SCI1_OutString("p\n");        // Broadcast to PC that PC mode is enabled.
    }

     /* Main Program Loop */
    while (1) {
    
        // Get scan or test mode
         
        cmd2LCD(0x01);        // Clear LCD display        
        cmd2LCD(0x80);        // LCD row 1
        putsLCD("1-Scan");
        cmd2LCD(0xC0);        // LCD row 2
        putsLCD("2-Test");          
        
        if (pc_mode == 1) {  
            SCI1_OutString("a\n");          // Broadcast to PC that the board is waiting for input.
            SCI1_OutString("'1'_-_Scanning_Mode______'2'_-_Testing_Mode\n");  
        }
        
        while (1) {           
            if (pc_mode == 0) {
                button_press = getkey();        
            } else {                               
                button_press = getc_buffer();
                getc_buffer();             // Clear newline
            }
            
            if (button_press == '1' || button_press == '2') {                 
                break;  
            } else if (pc_mode == 1) {
                SCI1_OutString("a\n");        // Broadcast to PC that the board is waiting for input.
                SCI1_OutString("'1'_-_Scanning Mode______'2'_-_Testing Mode\n"); 
                read_ptr = write_ptr;             // Clear buffer   
            }
        }
        
                
        if (button_press == '1') {
            
            /***** Scan Mode *****/

            float resolution;       // Change in angle between each measurement point
            int samples;            // Number of measurements averaged per point
            int scan_delay;         // Delay between each lidar measurement
            
                          
            // Get resolution
            cmd2LCD(0x01); // Clear LCD display              
            cmd2LCD(0x80);    // LCD row 1
            putsLCD("Resolution:");
            cmd2LCD(0xC0);    // LCD row 2
            putsLCD(">> ");
            
            while (1) {
                if (pc_mode == 1) {                
                    SCI1_OutString("a\n");        // Broadcast to PC that the board is waiting for input.
                    SCI1_OutString("Enter_scan_resolution:_\n");     
                }              
                getkey_string(1);   // Get string from keypad or PC, decimal point enabled
                resolution = atof(keypad_buffer);   // Convert the string to a number
                
                if (resolution >= 0.099999 && resolution <= 20) {
                    break;    
                }
            }                                                                           

                                   
            
            // Get samples
            
            cmd2LCD(0x01); // Clear LCD display              
            cmd2LCD(0x80);    // LCD row 1
            putsLCD("Samples:");
            cmd2LCD(0xC0);    // LCD row 2
            putsLCD(">> ");
            
            while (1) {
                if (pc_mode == 1) {                
                    SCI1_OutString("a\n");        // Broadcast to PC that the board is waiting for input.
                    SCI1_OutString("Enter_samples_per_point:_\n");    
                }              
                getkey_string(0); // Get string from keypad or PC, decimal point disabled
                samples = str2num(keypad_buffer);   // Convert the string to an integer
                
                if (samples >= 1 && samples <= 200) {
                    break;    
                }
            }
            
           
            
            // Get scan delay
            
            cmd2LCD(0x01); // Clear LCD display              
            cmd2LCD(0x80);    // LCD row 1
            putsLCD("Scan Delay:");
            cmd2LCD(0xC0);    // LCD row 2
            putsLCD(">> ");
            
            while (1) {
                if (pc_mode == 1) {                
                    SCI1_OutString("a\n");        // Broadcast to PC that the board is waiting for input.
                    SCI1_OutString("Enter_the_scan_delay_time_(ms):_\n");    
                }              
                getkey_string(0); // Get string from keypad or PC, decimal point disabled
                scan_delay = str2num(keypad_buffer);   // Convert the string to an integer
                
                if (scan_delay >= 0 && scan_delay <= 10000) {
                    break;                  
                }
            }
            
                        
            
            // Get Full Scan or Rectangle Scan
            cmd2LCD(0x01);            // Clear LCD display
            cmd2LCD(0x80);            // LCD row 1
            putsLCD("1-Full Scan");
            cmd2LCD(0xC0);            // LCD row 2
            putsLCD("2-Rectangle Scan");
            
                     
            while(1) {
                
                if (pc_mode == 0) {
                    button_press = getkey();        
                } else {
                    SCI1_OutString("a\n");        // Broadcast to PC that the board is waiting for input.
                    SCI1_OutString("1_-_Full_Scan______2_-_Rectangle_Scan\n");
                    button_press = getc_buffer();
                } 
                
                if (button_press == '1') {
                  
                    /*** Full Scan Mode ***/
                    find_edges(2*resolution, 2*samples);
                    full_scan(resolution, samples, scan_delay);
                    break;
                      
                } else if (button_press == '2') {
                  
                    /*** Rectangle Scan Mode ***/
                    //sweep(resolution, samples, scan_delay);
                    //while(1);
                    //find_edges(resolution, samples);
                    rectangle_scan(resolution, samples, scan_delay);
                    break;
                       
                }
            }
            
            getc_buffer();
        }
                                        
        else if (button_press == '2') {
        
            /***** Test Mode *****/
            
            cmd2LCD(0x01); // Clear LCD display
            cmd2LCD(0x80); // LCD row 1            
            putsLCD("1-Servo 2-LIDAR");
            cmd2LCD(0xC0);    // LCD row 2
            putsLCD("3-Gy  4-Ac  5-Ma");
            
            while (1) {             
                              
                if (pc_mode == 0) {
                    button_press = getkey();        
                } else {
                    SCI1_OutString("a\n");        // Broadcast to PC that the board is waiting for input.
                    SCI1_OutString("1_-_Servo_Test____2_-_LIDAR_Test____3_-_Gyroscope_Test____4_-_Accelerometer_Test____5_-_Magnetometer\n");
                    button_press = getc_buffer();
                }
            
                if (button_press == '1') {                
                                                
                    /*** Servo Test Mode ***/    
                    servo_test();
     
                } else if (button_press == '2') {
                
                    /*** LIDAR Test Mode ***/             
                    lidar_test();
                    
                } else if (button_press == '3') {
                
                    /*** Gyroscope Test Mode ***/             
                    gyro_test();
                    
                } else if (button_press == '4') {
                
                    /*** Accelerometer Test Mode ***/             
                    accel_test();
                    
                } else if (button_press == '5') {
                
                    /*** Magnetometer Test Mode ***/             
                    magnet_test();
                    
                }
            }
        }    
    }    
}


/*! Executes 4 inward LIDAR scans from each side to detect the approximate edges of the shape. The edge angles are stored in the global array shape_edges. */
void find_edges(float resolution, float samples) {
  
    float prev_distance;    // Previous distance value

    float pan_angle = 60;   // Current angle of pan servo     
    float tilt_angle = 90;  // Current angle of tilt servo
    
    float tolerance = resolution + 3;
    
    cmd2LCD(0x01); // Clear LCD display
    
    
    // Left edge
    pan_servo(pan_angle);     // Move to initial pan angle
    tilt_servo(tilt_angle);   // Move to initial tilt angle
    
    delay_ms(200);    
    distance = get_distance(samples);
      
    for (pan_angle = 60; pan_angle <= 120; pan_angle += resolution) {              
        pan_servo(pan_angle);
        
        prev_distance = distance;    
        distance = get_distance(samples);

        if (prev_distance - distance >= 100 && distance <= 1200 && distance >= 800) {
            shape_edges[0] = pan_angle - tolerance;
            break;
        }                            
        display_angle(pan_angle, tilt_angle); 
    }
    
    // Right edge
    pan_servo(120);     // Move to initial pan angle
    tilt_servo(90);   // Move to initial tilt angle    
    distance = get_distance(samples);
    for (pan_angle = 120; pan_angle >= 60; pan_angle -= resolution) {              
        pan_servo(pan_angle);
        
        prev_distance = distance;    
        distance = get_distance(samples);
        
        if (prev_distance - distance >= 100 && distance <= 1200 && distance >= 800) {
            shape_edges[1] = pan_angle + tolerance;
            break;
        }                          
        display_angle(pan_angle, tilt_angle); 
    }
    
    // Bottom edge
    pan_servo((shape_edges[0]+shape_edges[1])/2 + 2);     // Move to average pan_angle of the edges plus an offset to account for the stand
    tilt_servo(65);   // Move to initial tilt angle    
    distance = get_distance(samples);
    for (tilt_angle = 65; tilt_angle <= 115; tilt_angle += resolution) {              
        tilt_servo(tilt_angle);
        
        prev_distance = distance;    
        distance = get_distance(samples);
        
        if (prev_distance - distance >= 100 && distance <= 1200 && distance >= 800) {
            shape_edges[2] = tilt_angle - tolerance;
            break;
        }                          
        display_angle(pan_angle, tilt_angle); 
    }
    
    // Top edge
    pan_servo((shape_edges[0]+shape_edges[1])/2 - 2);     // Move to average pan_angle of the edges plus an offset to account for the stand
    tilt_servo(115);   // Move to initial tilt angle    
    distance = get_distance(samples);
    for (tilt_angle = 115; tilt_angle >= 65; tilt_angle -= resolution) {              
        tilt_servo(tilt_angle);
        
        prev_distance = distance;    
        distance = get_distance(samples);
        
        if (prev_distance - distance >= 100 && distance <= 1200 && distance >= 800) {
            shape_edges[3] = tilt_angle + tolerance;
            break;
        }                          
        display_angle(pan_angle, tilt_angle); 
    }
    if (pc_mode == 1) {         
      SCI1_OutString("-1\n");  // Output termination string       
    }     
}

                                               
/*! Performs one LIDAR sweep using the pan servo. This function is used for performance testing. */
void sweep(float resolution, int samples, int scan_delay) {

    float pan_angle = 40;   // Current angle of pan servo     
    float tilt_angle = 90;  // Current angle of tilt servo
    
    pan_servo(pan_angle);     // Move to initial pan angle
    tilt_servo(tilt_angle);   // Move to initial tilt angle
    
    cmd2LCD(0x01); // Clear LCD display
    
    if (pc_mode == 1) {
        SCI1_OutString("d\n");        // Broadcast to PC to begin reading data.
        SCI1_OutString("sweep\n"); // Send scanning mode to PC
        memset(string,0,sizeof(string));   // Clear the string
        sprintf(string, "%.1f", resolution);    // Convert the resolution value to a string
        SCI1_OutString(string); // Send resolution to PC
        SCI1_OutChar('\n');             
    }  
  
    for (pan_angle = 75; pan_angle <= 105; pan_angle += resolution) {
              
        pan_servo(pan_angle);

        if (pan_angle == 75) {
            delay1s();  
        }
        delay_ms(scan_delay);          // User configurable delay            
        distance = get_distance(samples);                       
        
        display_angle(pan_angle, tilt_angle);
        
        memset(string,0,sizeof(string));   // Clear the string
        sprintf(string, "%.0f", distance);    // Convert the distance value to a string 
        cmd2LCD(0xC0);  // LCD row 2
        putsLCD("Dist: ");                 
        putsLCD(string);
        putsLCD("mm");       
          
        // Send data to serial if PC mode
        if (pc_mode == 1) {                              
            send_point(pan_angle, tilt_angle, distance);                                                  
        }
    }
    
    if (pc_mode == 1) {         
      SCI1_OutString("-1\n");  // Output termination string       
    }
} 


/*! Execute the rectangle scan algorithm using the specified resolution, sample rate and scan delay. */
void rectangle_scan(float resolution, int samples, int scan_delay) {

    float prev_distance;    // Previous distance value

    float pan_angle = 60;   // Current angle of pan servo     
    float tilt_angle = 90;  // Current angle of tilt servo
    
    int stage = 0;
    
    pan_servo(pan_angle);     // Move to initial pan angle
    tilt_servo(tilt_angle);   // Move to initial tilt angle
    delay_ms(200);
    
    cmd2LCD(0x01); // Clear LCD display
    
    if (pc_mode == 1) {
        SCI1_OutString("d\n");        // Broadcast to PC to begin reading data.
        SCI1_OutString("rectangle\n"); // Send scanning mode to PC
        memset(string,0,sizeof(string));   // Clear the string
        sprintf(string, "%.1f", resolution);    // Convert the resolution value to a string
        SCI1_OutString(string); // Send resolution to PC
        SCI1_OutChar('\n');             
    }                              
    
    distance = get_distance(samples);
  
    for (pan_angle = 60; pan_angle <= 120; pan_angle += resolution) {
              
        pan_servo(pan_angle);
        
        prev_distance = distance;
        delay_ms(scan_delay);    
        distance = get_distance(samples);                                     
        
        display_angle(pan_angle, tilt_angle);
        
        memset(string,0,sizeof(string));   // Clear the string
        sprintf(string, "%.0f", distance);    // Convert the distance value to a string 
        cmd2LCD(0xC0);  // LCD row 2
        putsLCD("Dist: ");                 
        putsLCD(string);
        putsLCD("mm");       
          
        // Send data to serial if PC mode
        if (pc_mode == 1) {                              
            send_point(pan_angle, tilt_angle, distance);                                                  
        }
        
        if (prev_distance - distance >= 100 && distance <= 1200 && distance >= 800) {
            shape_edges[0] = pan_angle;
            stage++;
        }
        //if (stage == 1 && distance - prev_distance >= 100 && prev_distance <= 1200 && prev_distance >= 800) {
        if (stage == 1 && ( distance > 1200 || distance < 800 ) && distance - prev_distance >= 100) {
            shape_edges[1] = pan_angle - resolution;
            stage++;
            break;
        }  
    }
    
    pan_angle = (shape_edges[0] + shape_edges[1])/2 + 3;
    pan_servo(pan_angle);
    tilt_servo(65);
    delay_ms(200);
    distance = get_distance(samples);
    
    for (tilt_angle = 65; tilt_angle <= 115; tilt_angle += resolution) {
              
        tilt_servo(tilt_angle);

        prev_distance = distance;
        delay_ms(scan_delay);    
        distance = get_distance(samples);
        
        display_angle(pan_angle, tilt_angle);
        
        memset(string,0,sizeof(string));   // Clear the string
        sprintf(string, "%.0f", distance);    // Convert the distance value to a string 
        cmd2LCD(0xC0);  // LCD row 2
        putsLCD("Dist: ");                 
        putsLCD(string);
        putsLCD("mm");       
          
        // Send data to serial if PC mode
        if (pc_mode == 1) {                              
            send_point(pan_angle, tilt_angle, distance);                                                  
        }
        
        if (stage == 2 && prev_distance - distance >= 100 && distance <= 1200 && distance >= 800) {
            shape_edges[2] = tilt_angle;
            stage++;        
        }
        //if (stage == 3 && distance - prev_distance >= 100 && prev_distance <= 1200 && prev_distance >= 800) {
        if (stage == 3 && ( distance > 1200 || distance <= 800 ) && distance - prev_distance >= 100) {        
            shape_edges[3] = tilt_angle - resolution;
            break;
        }   
    }  
    
    if (pc_mode == 1) {         
      SCI1_OutString("-1\n");  // Output termination string       
    }
    
          
}


/*! Execute a full LIDAR scan of an area determined by the global array shape_edges using the specified resolution, sample rate and scan delay. */
void full_scan(float resolution, int samples, int scan_delay) {

    float pan_angle;   // Current angle of pan servo     
    float tilt_angle;  // Current angle of tilt servo
    
    const float left_edge = shape_edges[0];
    const float right_edge = shape_edges[1];
    const float bottom_edge = shape_edges[2];
    const float top_edge = shape_edges[3];
    
    
    int direction = 1; // Initialise pan movement in positive direction
        
    cmd2LCD(0x01); // Clear LCD display
    
    if (pc_mode == 1) {
        SCI1_OutString("d\n");        // Broadcast to PC to begin reading data.
        SCI1_OutString("full\n"); // Send scanning mode to PC
        memset(string,0,sizeof(string));   // Clear the string
        sprintf(string, "%.1f", resolution);    // Convert the resolution value to a string
        SCI1_OutString(string); // Send resolution to PC
        SCI1_OutChar('\n');  
    }
    

    for (tilt_angle = bottom_edge; tilt_angle <= top_edge;  tilt_angle += resolution) {
            
        tilt_servo(tilt_angle);
      
        for (pan_angle = left_edge; pan_angle <= right_edge; pan_angle += resolution) {
          
            float pan_correct = pan_angle;            
            
            if (direction < 0) {                // If direction is negative             
                pan_correct = 180 - pan_angle;  // Invert the angle to sweep in the reverse direction
            }

            pan_servo(pan_correct);
            if (pan_angle == shape_edges[0]) {
                delay_ms(200);  
            }
            delay_ms(scan_delay);            
            distance = get_distance(samples);                       
            
            display_angle(pan_correct, tilt_angle);   // Display the servo angles on the LCD

            
            memset(string,0,sizeof(string));   // Clear the string
            sprintf(string, "%.0f", distance);    // Convert the distance value to a string 
            cmd2LCD(0xC0);  // LCD row 2
            putsLCD("Dist: ");                 
            putsLCD(string);
            putsLCD("mm");
              
            // Send data to serial if PC mode
            if (pc_mode == 1) {                              
                send_point(pan_correct, tilt_angle, distance);                                                  
            }
        }
        
        //direction *= -1;      // Uncomment this to scan back and forth instead of only scanning in one direction                            
    }
    
    if (pc_mode == 1) {         
      SCI1_OutString("-1\n");  // Output termination character       
    }
          
}


/*! Testing program for the pan and tilt servo motors. */
void servo_test(void) {

    float pan_angle = 90;
    float tilt_angle = 90;

    float angle; // Variable to store the angle
    char button_press; // Variable to store the ASCII value of a button press
                    
    cmd2LCD(0x01); // Clear LCD display
    
    pan_servo(pan_angle);
    tilt_servo(tilt_angle);
    display_angle(pan_angle, tilt_angle);
    
    cmd2LCD(0xC0);  // LCD row 2
    putsLCD(">> ");
    
    
    if (pc_mode == 1) {
        SCI1_OutString("a\n");        // Broadcast to PC that the board is waiting for input.
        SCI1_OutString("Enter_angle:_\n");      
    }

    while (1) {         
        
        if (pc_mode == 0) {
            button_press = getkey();        
        } else {                
            button_press = getc_buffer();
        }
        
        if (button_press == 'A' || button_press == 'p') {     // 
                                
            pan_angle = atof(keypad_buffer);   // Convert the string to an integer angle
            
            if (pan_angle >= 0 && pan_angle <= 180) {       // Only move the servo if the angle is between 0 and 180 degrees        
                pan_servo(pan_angle);
                display_angle(pan_angle, tilt_angle);
            } else if (pc_mode == 1) {
                read_ptr = write_ptr;                             // Clear the input buffer
                SCI1_OutString("a\n");        // Broadcast to PC that the board is waiting for input.
                SCI1_OutString("Enter_angle:_\n");   
            }
                         
            memset(keypad_buffer,0,sizeof(keypad_buffer));    // Clear the keypad buffer
            buffer_ptr = keypad_buffer;                       // Reset the pointer
            cmd2LCD(0xC0);    // LCD row 2
            putsLCD("                "); // Clear current row
            cmd2LCD(0xC0);    // LCD row 2
            putsLCD(">> "); 
                          
          
        } else if (button_press == 'D' || button_press == 't') {                     
            
            tilt_angle = atof(keypad_buffer);   // Convert the string to an integer angle
            
            if (tilt_angle >= 0 && tilt_angle <= 180) {       // Only move the servo if the angle is between 0 and 180 degrees        
                tilt_servo(tilt_angle);
                display_angle(pan_angle, tilt_angle);
            } else if (pc_mode == 1) {
                read_ptr = write_ptr;                             // Clear the input buffer
                SCI1_OutString("a\n");        // Broadcast to PC that the board is waiting for input.
                SCI1_OutString("Enter_angle:_\n");   
            }            
            
            memset(keypad_buffer,0,sizeof(keypad_buffer));    // Clear the buffer
            buffer_ptr = keypad_buffer;                       // Reset the pointer
            cmd2LCD(0xC0);    // LCD row 2
            putsLCD("                "); // Clear current row
            cmd2LCD(0xC0);    // LCD row 2
            putsLCD(">> "); 
            
        } else if (button_press == '*' || button_press == '.') {                  
            
                putcLCD('.');                                     // Print the pressed character
                *buffer_ptr = '.';                                // Store the key in the current buffer pointer address
                buffer_ptr++; 
             
        } else if (button_press >= '0' && button_press <= '9') {
        
            putcLCD(button_press);														// Print the pressed character
            *buffer_ptr = button_press;                       // Store the key in the current buffer pointer address
            buffer_ptr++;
                                                 // Increment the buffer pointer
        } else if (button_press == 'C') {  // Backspace key
            buffer_ptr--;                                     // Decrement the buffer pointer
                          *buffer_ptr = '';                                 // Erase the character at the pointer address
            cmd2LCD(0x10);                                    // Decrement cursor
            putcLCD(' ');                                     // Overwrite the current value with an empty space
            cmd2LCD(0x10);                                    // Decrement cursor 
                               
        } else if (button_press == '\n') {
        
            read_ptr = write_ptr;                             // Clear the input buffer
            SCI1_OutString("a\n");        // Broadcast to PC that the board is waiting for input.
            SCI1_OutString("Enter_angle:_\n");
            
            memset(keypad_buffer,0,sizeof(keypad_buffer));    // Clear the buffer
            buffer_ptr = keypad_buffer;
            cmd2LCD(0xC0);    // LCD row 2
            putsLCD("                "); // Clear current row
            cmd2LCD(0xC0);    // LCD row 2
            putsLCD(">> ");              
        }
    }
}


/*! Testing program for the LIDAR. */
void lidar_test(void) {

    int samples;

    // Get samples
            
    cmd2LCD(0x01); // Clear LCD display              
    cmd2LCD(0x80);    // LCD row 1
    putsLCD("Samples:");
    cmd2LCD(0xC0);    // LCD row 2
    putsLCD(">> ");
    
    while (1) {
        if (pc_mode == 1) {                
            SCI1_OutString("a\n");          // Broadcast to PC that the board is waiting for input.
            SCI1_OutString("Enter_samples_per_point:_\n");    
        }              
        getkey_string(0);                   // Get string from keypad or PC, decimal point disabled
        samples = str2num(keypad_buffer);   // Convert the string to an integer
        
        if (samples >= 1 && samples <= 200) {    // If value of samples is within valid range
            break;                               // Exit the loop
        }
    }
            
    
    
    cmd2LCD(0x01); // Clear LCD display
    cmd2LCD(0x80); // LCD row 1
    
    if (pc_mode == 1) { 
        SCI1_OutString("l\n");        // Broadcast to PC to enter LIDAR test mode.
    }

    while (1) {
   
        distance = get_distance(10);          
        
        memset(string,0,sizeof(string));   // Clear the string        
        sprintf(string, "%.0f", distance);    // Convert the integer distance value to a string
        cmd2LCD(0x01);          // Clear LCD display 
        cmd2LCD(0x80);          // LCD row 1
        putsLCD("Distance: ");                 
        putsLCD(string);
        putsLCD("mm");
        
        if (pc_mode == 1) {
            SCI1_OutString(string); 
            SCI1_OutChar('\n');
        }
        
        delay_ms(500);
    }
      
}


/*! Testing program for the gyroscope. */
void gyro_test(void) {
    int g_pan, g_tilt;
    
    cmd2LCD(0xC0);    // LCD row 2
    putsLCD("                "); // Clear current row
    cmd2LCD(0xC0);    // LCD row 2
    putsLCD("Gyroscope");
    
    if (pc_mode == 1) {
        SCI1_OutString("g\n");
    }   
    
    while (1) {        
        gyro_imu(&g_pan, &g_tilt);
        display_angle((float)g_pan, (float)g_tilt);
        if (pc_mode == 1) {
            sprintf(string, "%d\n", g_pan);    
            SCI1_OutString(string);
            sprintf(string, "%d\n", g_tilt);
            SCI1_OutString(string);
        }
        delay_ms(30);
    }      
}


/*! Testing program for the accelerometer. */
void accel_test(void) {
    int a_tilt;
    
    cmd2LCD(0xC0);    // LCD row 2
    putsLCD("                "); // Clear current row
    cmd2LCD(0xC0);    // LCD row 2
    putsLCD("Accelerometer");
    
    if (pc_mode == 1) {
        SCI1_OutString("t\n");
    }
    
    while (1) {        
        accel_imu(&a_tilt);
        display_angle(0.0, (float)a_tilt);
        if (pc_mode == 1) {    
            sprintf(string, "%d\n", a_tilt);    
            SCI1_OutString(string);
        }
        delay_ms(30);
    }      
}


/*! Testing program for the magnetometer. */
void magnet_test(void) {
    int m_pan;
    
    cmd2LCD(0xC0);    // LCD row 2
    putsLCD("                "); // Clear current row
    cmd2LCD(0xC0);    // LCD row 2
    putsLCD("Magnetometer");
    
    if (pc_mode == 1) {
        SCI1_OutString("m\n");
    }   
    
    while (1) {        
        magnet_imu(&m_pan);
        display_angle((float)m_pan, 0.0);
        if (pc_mode == 1) {             
            sprintf(string, "%d\n", m_pan);    
            SCI1_OutString(string); 
        }
        delay_ms(30);
    }      
}


/*! Converts a string of numerical data to an integer value. */
int str2num(char* string) {
    int i;
  	int total = 0;
  	int multiplier;
  	int mult_count;
  	for (i=0; i<strlen(string); i++) {
    		multiplier = 1;
    		for (mult_count = 0; mult_count < strlen(string)-1-i; mult_count++) {
    			multiplier *= 10;			
    		}
        total += (string[i]-48) * multiplier;            
    }
  	return total;
}


/*! Reads a single button press from the HCS12 keypad and returns the ASCII value of the key pressed. */
char getkey(void){                          

    char key; // Variable to store the pressed digit
    
    const unsigned char keypad[4][4] =
    {
    '1','2','3','A',
    '4','5','6','B',
    '7','8','9','C',
    '*','0','#','D'
    };
    
    unsigned char column,row;          
    
    DDRA = 0x0F;                           //Make rows input and columns output                  
 
    while(1){                         

      do{                                 
         PORTA = PORTA | 0x0F;            // Set columns high
         row = PORTA & 0xF0;              // Read rows
      }while(row == 0x00 && pc_mode == 0);                // Wait until key pressed or pc_mode initialised

      if (pc_mode == 1) {
          return;    // Leave this function if in PC mode
      }

      do{                               
         do{                             
            mSDelay(1);                  
            row = PORTA & 0xF0;           // Read rows
         }while(row == 0x00);             // Check for key press
         
         mSDelay(15);                     // Wait for debounce to verify key press
         row = PORTA & 0xF0;
      }while(row == 0x00);                // Check key press again

      while(1){                           
         PORTA &= 0xF0;                   // Clear columns
         PORTA |= 0x01;                   // Set column 0 high
         row = PORTA & 0xF0;              // Read rows
         if(row != 0x00){                 // If key is in column 0
            column = 0;
            break;                        
         }
         PORTA &= 0xF0;                   // Clear columns
         PORTA |= 0x02;                   // Set column 1 high
         row = PORTA & 0xF0;              // Read rows
         if(row != 0x00){                 // If key is in column 1
            column = 1;
            break;                        
         }

         PORTA &= 0xF0;                   // Clear columns
         PORTA |= 0x04;                   // Set column 2 high
         row = PORTA & 0xF0;              // Read rows
         if(row != 0x00){                 // If key is in column 2
            column = 2;
            break;                        
         }
         PORTA &= 0xF0;                   // Clear columns
         PORTA |= 0x08;                   // Set column 3 high
         row = PORTA & 0xF0;              // Read rows
         if(row != 0x00){                 // If key is in column 3
            column = 3;
            break;                      
         }
         row = 0;                      
      break;                             
      }                                  

      if(row == 0x10){
         key = keypad[0][column];         
 
      }
      else if(row == 0x20){
         key = keypad[1][column];
 
      }
      else if(row == 0x40){
         key = keypad[2][column];
 
      }
      else if(row == 0x80){
         key = keypad[3][column];
 
      }
      do{
         mSDelay(15);
         PORTA = PORTA | 0x0F;            // Set all columns high
         row = PORTA & 0xF0;              // Read rows
      }while(row != 0x00);                // Make sure no buttons are currently pressed
      
      return key;
   }                                     
}   


/*! Reads in a string from either the keypad_buffer (if in board mode) or the input_buffer (if in PC mode). */
void getkey_string(int float_mode) {

    memset(keypad_buffer,0,sizeof(keypad_buffer));    // Clear the keypad buffer
 
    while (1) {          
       
        if (pc_mode == 0) {
            button_press = getkey();        
        } else {                    
            button_press = getc_buffer();
        }    
        
        if (button_press == 'D' || button_press == '\n') {  // Enter key                   
            
            buffer_ptr = keypad_buffer;       // Reset the pointer
            read_ptr = write_ptr;             // Clear the input buffer
            
            cmd2LCD(0xC0);    // LCD row 2
            putsLCD("                "); // Clear current row
            cmd2LCD(0xC0);    // LCD row 2
            putsLCD(">> "); 
                                
            return;                    
            
              
        } else if (button_press >= '0' && button_press <= '9') {
        
            putcLCD(button_press);                            // Print the pressed character
            *buffer_ptr = button_press;                       // Store the key in the current buffer pointer address
            buffer_ptr++;                                     // Increment the buffer pointer
            
        } else if ((button_press == '*' || button_press == '.') && float_mode == 1) {    // Enable decimal point input if float_mode is true              
        
            putcLCD('.');                                     // Print the pressed character
            *buffer_ptr = '.';                                // Store the key in the current buffer pointer address
            buffer_ptr++;                                     // Increment the buffer pointer
          
        } else if (button_press == 'C') {  // Backspace key
            buffer_ptr--;                                     // Decrement the buffer pointer
            *buffer_ptr = '';                                 // Erase the character at the pointer address
            cmd2LCD(0x10);                                    // Decrement cursor
            putcLCD(' ');                                     // Overwrite the current value with an empty space
            cmd2LCD(0x10);                                    // Decrement cursor                    
        }
    } 
}


/*! Delay for a specified number of milliseconds. */
void delay_ms(unsigned int itime){
    unsigned int i; unsigned int j;
    for(i=0;i<itime;i++)
       for(j=0;j<4000;j++);
}


/*! Reads a single character from the global buffer array input_buffer. */
char getc_buffer(void) {

    char data;
    
    if (read_ptr == write_ptr) {                         // If there is nothing new to read        
        read_ptr = input_buffer;                        // Reset read_ptr to the beginning of the buffer
        write_ptr = input_buffer;                       // Reset write_ptr to the beginning of the buffer      
    }          
    
    while (read_ptr == write_ptr);                      // Wait for data to appear in the buffer if there is none

    data = *read_ptr;       // Read the byte in the buffer at the location of read_ptr
    read_ptr++;             // Increment read_ptr in preparation for the next read
    
    return data;  
}


/*! Sends a data point (pan, tilt, distance) out through the serial interface. */
void send_point(float pan, float tilt, float distance) {
    memset(string,0,sizeof(string));   // Clear the string
    sprintf(string, "%.1f", pan);      // Convert the number to a string
    SCI1_OutString(string);
    SCI1_OutChar('\n');
    
    memset(string,0,sizeof(string));   // Clear the string
    sprintf(string, "%.1f", tilt);     // Convert the number to a string
    SCI1_OutString(string);
    SCI1_OutChar('\n');
    
    memset(string,0,sizeof(string));   // Clear the string
    sprintf(string, "%.0f", distance); // Convert the number to a string
    SCI1_OutString(string);
    SCI1_OutChar('\n');  
}


/*! Displays the current pan and tilt angle on to the first row of the LCD display. */
void display_angle(float pan, float tilt) {

    char pan_string[10], tilt_string[10];
    
    cmd2LCD(0x80);    // LCD row 1
    putsLCD("                "); // Clear current row
    cmd2LCD(0x80);    // LCD row 1
               
    sprintf(pan_string, "%.1f", pan);    // Convert the distance value to a string
    cmd2LCD(0x80);    // LCD row 1
    putsLCD("P:");                 
    putsLCD(pan_string);
            
    sprintf(tilt_string, "%.1f", tilt);    // Convert the distance value to a string
    cmd2LCD(0x88);    // LCD row 1
    putsLCD("T:");                 
    putsLCD(tilt_string);
}


/*! Initialises timer output compare on TC3 and enables an interupt to cycle through the 7-segment displays. */
void segment_init(void) {    
    DDRP = 0xFF;    // Configure Port P (7-seg enable) as output
    DDRB = 0xFF;    // Configure Port B (7-seg) as output
    
    TSCR1 = 0x90;   //  Timer enabled, Fast flag clear enabled
    TSCR2 = 0x00;   //  Timer Overflow Interrupt disabled, prescaler set to 1 
    
    TIOS |= 0x08;   // TC3 set up as output compare   
    
    TIE |= 0x08;    // Output Compare Interrupt enabled on Timer Channel 3       
}

/*! Disables TC3 interrupts and disables all 7-segment displays. */
void segment_stop(void) {
    PTP = 0xFF;     // Disable all 7-segs
    PORTB = 0x00;   // Turn off all 7-segs
    TIE &= ~0x08;   // Disable Timer Channel 3 interrupts  
}


/*! Interrupt to cycle through 7-segment displays. */
interrupt 11 void timer3_ISR(void) {

    int i;
    int distance_copy = (int)distance;     // Integer copy of the current distance value
    int digit;
            
    for (i=0; i<3-segment_count; i++) {
        distance_copy /= 10;      
    }
    digit = distance_copy%10;              // Get the current digit to display
    
    PTP = enable_disp[segment_count];      // Enable the current 7-segment
    PORTB = digits[digit];                 // Display the number on the enabled 7-segment
    
    if (segment_count == 3) {     // If currently on final segment
        segment_count = 0;        // Reset back to first segment
    } else {
        segment_count++;          // Increment to the next segment
    }
    TC3 = TCNT + 24000;           // Return to this interrupt after a delay
}


/*! When a serial interrupt is detected from incoming data, the data is stored into a global buffer array input_buffer. */
interrupt 21 void serial_ISR(void) {

    if (pc_mode == 0) {                            // If serial interrupt occurs when not in PC mode
        pc_mode = 1;                               // Set the program to PC mode
        DDRJ = 0xFF;                               // Light up all LEDs
        DDRB = 0xFF;                               //
        PTJ = 0x00;                                //
        PORTB = 0xFF;                              //
        
    } else {
    
        if (SCI1SR1 & 0x20) {                       // If data is detected
                    
            *write_ptr = SCI1DRL;                   // Store the data in the buffer at the write_ptr location
            write_ptr++;                            // Increment the buffer write_ptr in preparation for the next value
        
        }      
    }
}