#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */


#define LCD_DAT PORTK       // Port K drives LCD data pins, E, and RS
#define LCD_DIR DDRK        // Direction of LCD port
#define LCD_E 0x02          // E signal
#define LCD_RS 0x01         // RS signal
#define LCD_E_RS 0x03       // Assert both E and RS signals



/*! Send a command to the LCD display for perform a certain action. */
void cmd2LCD (char cmd){  

    char temp;
    temp = cmd;             // Save a copy of the command
    cmd &= 0xF0;            // Clear out the lower 4 bits
    LCD_DAT &= (~LCD_RS);   // Select LCD instruction register
    LCD_DAT |= LCD_E;       // Pull E signal to high
    cmd >>= 2;              // Shift to match LCD data pins
    LCD_DAT = cmd | LCD_E;  // Output upper 4 bits, E, and RS
    asm ("nop");            // Dummy statements to lengthen E
    asm ("nop"); 
    asm ("nop");
    LCD_DAT &= (~LCD_E);    // Pull E signal to low
    cmd = temp & 0x0F;      // Extract the lower 4 bits
    LCD_DAT |= LCD_E;       // Pull E to high
    cmd <<= 2;              // Shift to match LCD data pins
    LCD_DAT = cmd | LCD_E;  // Output upper 4 bits, E, and RS
    asm("nop");             // Dummy statements to lengthen E
    asm("nop"); 
    asm("nop");
    LCD_DAT &= (~LCD_E);    // Pull E-clock to low
    delay1ms();             // Wait until the command is complete

}


/*! Initialise the LCD display. */
void lcd_init(void){

    LCD_DIR = 0xFF;         // Data direction port set as output
    delay1ms();
    cmd2LCD(0x28);          // 2 line display, 4-bit transfer
    cmd2LCD(0x0C);          // Display on, cursor off
    cmd2LCD(0x06);
    cmd2LCD(0x01);          // Clear display

    delay1ms();
}

/*! Display a string on the LCD display. */
void putcLCD(char cx) {

    char temp;
    temp = cx;
    LCD_DAT |= LCD_RS;      // Select LCD data register
    LCD_DAT |= LCD_E;       // Pull E signal to high
    cx &= 0xF0;             // Clear the lower 4 bits
    cx >>= 2;               // Shift to match the LCD data pins
    LCD_DAT = cx|LCD_E_RS;  // Output upper 4 bits, E, and RS
    asm("nop");             // Dummy statements to lengthen E
    asm("nop"); 
    asm("nop");
    LCD_DAT &= (~LCD_E);    // Pull E to low
    cx = temp & 0x0F;       // Get the lower 4 bits
    LCD_DAT |= LCD_E;       // Pull E to high
    cx <<= 2;               // Shift to match the LCD data pins
    LCD_DAT = cx|LCD_E_RS;  // Output lower 4 bits, E, and RS
    asm("nop");             // Dummy statements to lengthen E
    asm("nop"); 
    asm("nop");
    LCD_DAT &= (~LCD_E);    // Pull E to low
    delay1ms();
    
}


/*! Display a character on the LCD display. */
void putsLCD (char *ptr){

  while (*ptr){
  
    putcLCD (*ptr);
    ptr++;
  }
}