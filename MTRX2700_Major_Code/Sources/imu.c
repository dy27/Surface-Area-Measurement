// IMU Information 
//
// Initialization (void) 
// -> Gyro
// -> Accelerometer
// -> Magnetometer
//
// Gyro (&g_pan ,&g_tilt)
// -> Returns tilt and pan angles
//
// Accelerometer (&a_tilt)
// -> Returns tilt angle
//
// Magnetometer (&m_pan)
// -> Returns pan angle
// (Currently only works for tilt angle of 0)



#include <hidef.h>      /* common defines and macros */
#include <math.h>
#include "derivative.h"      /* derivative-specific definitions */
#include "iic.h"  
#include "pll.h"
#include "sci1.h"

#include "l3g4200.h"  // register's definitions    ( not used by ed )


volatile uint8_t alarmSignaled1 = 0;  /* Flag set when alarm 1 signaled */

volatile uint16_t currentTime1 = 0;   /* variables private to timeout routines */
uint16_t alarmTime1 = 0;
volatile uint8_t alarmSet1 = 0;



void setAlarm1(uint16_t msDelay1); // Not sure if used
void delay1(uint16_t msDelay1);
void Init_TC6 (void);

#define gyro_wr 0xD2
#define gyro_rd 0xD3



#define accel_wr 0xA6    
#define accel_rd 0xA7    
#define ADXL345_TO_READ 6
 
#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATAX0 0x32
#define ADXL345_DATA_FORMAT 0x31
 
#define ADXL345_OFSX 0x1E
#define ADXL345_OFSY 0x1F
#define ADXL345_OFSZ 0x20
 
#define ALPHA 0.5

#define magnet_wr  0x3C
#define magnet_rd  0x3D

#define HM5883_MODE_REG 0x02
#define HM5883_DATAX0 0x03

#define TO_RAD 57.29577    // 180/(2*pi)
#define PI 3.1415

#define BUFF_SIZE	100

char buff[BUFF_SIZE];
int gxraw[BUFF_SIZE];
int gyraw[BUFF_SIZE],gzraw[BUFF_SIZE];	

int axraw[BUFF_SIZE];
int ayraw[BUFF_SIZE],azraw[BUFF_SIZE];	

int mxraw[BUFF_SIZE];
int myraw[BUFF_SIZE],mzraw[BUFF_SIZE];	



int k;



// Overall Init

void init_imu(void);

// Accel        
void accel_imu(int *a_tilt);
void adxl345_getrawdata(int *axraw, int *ayraw, int *azraw);
void accel_init(void);

// Magnet
void magnet_imu(int *m_pan);
void hm5883_getrawdata(int *mxraw, int *myraw, int *mzraw);
void magnet_init(void);

// Gyro
void gyro_imu(double *g_pan, double *g_tilt);
void l3g4200d_getrawdata(int *gxraw, int *gyraw, int *gzraw);
void gyro_init(void); 



int gxnew, gynew, gznew, gxold, gyold, gzold; // Gyro

signed short gxAngle, gyAngle;  // Gyro

int a_tilt, m_pan;   // Change to a_tilt + m_pan

double xy_axes, xz_mag, yz_mag; // Redundant

float dt, interval;             // Gyro

int g_pan, g_tilt; // Change to g_pan + g_tilt

double atan2(double x, double z);
double sqrt(double x);



/*! Initialises the settings for the gyroscope, accelerometer and magnetometer. */
void init_imu(void) {

 
    PLL_Init();  // make sure we are runnign at 24 Mhz


    EnableInterrupts;

    //This program will send the gyro, accelerometer adn magnetometer data
    // to a serial port
    // You can connect the serial port, set it to 9600 bauds 


    //Init_TC6();

    iicinit();


    gyro_init();     // l3g4200 setup
    accel_init();
    magnet_init();
 
 
}
 

/*! Get a measurement from the L3G4200d gyroscope sensor. */ 
void gyro_imu(int *g_pan, int *g_tilt){
  
    l3g4200d_getrawdata( &gxraw, &gyraw, &gzraw) ;        // read data
   

    gxnew = (signed short) gxraw[0];
    gxnew = gxnew - 210;          // Offset Balance     

    if (abs(gxnew) < 100) {       // High pass filter
      gxnew = 0;
    }
     
    dt = 0.0657;
    
    interval = (float)((gxold + gxnew)/2) * dt;
    
    gxAngle = gxAngle + (signed short)interval;
     
    gxold = gxnew; 
                                                                      //  0.00875 gyro constant
    *g_pan = gxAngle * 0.00875;
    
    
    gynew = (signed short) gyraw[0];
    gynew = gynew - 675;          // Offset Balance

    if (abs(gynew) < 100) {       // High pass filter
      gynew = 0;    
    }

    dt = 0.066;
    
    interval = (float)((gyold + gynew)/2) * dt;
    
    gyAngle = gyAngle + (signed short)interval;
     
    gyold = gynew; 
                                                                   
    *g_tilt = gyAngle * 0.00875;
  
}
 
 
/*! Get a measurement from the ADCL345 accelerometer sensor. */
void accel_imu(int *a_tilt) {
  
    adxl345_getrawdata( &axraw, &ayraw, &azraw) ;      // Read data

    *a_tilt = atan2(azraw[0],axraw[0]) * TO_RAD;        // Convert to Angle

}

/*! Get a measurement from the HM5883_magnetometer sensor. */
void magnet_imu(int *m_pan) {
  
    hm5883_getrawdata(&mxraw, &myraw, &mzraw);

    *m_pan = (atan2(mxraw[0], mzraw[0]) * TO_RAD -68) * 3.75;
 
} 



  
//   ******************  END Main   *****************


// Magnetometer

void magnet_init(void){
  
  int res1; 
  res1=iicstart(magnet_wr);
  res1=iictransmit(HM5883_MODE_REG );  // 
  res1=iictransmit(0x00 );
  iicstop(); 
 
}


void hm5883_getrawdata(int *mxraw, int *myraw, int *mzraw){
  
 uint8_t i = 0;
 uint8_t buff[6];
 int res1;
	
 res1=iicstart(magnet_wr);
 res1=iictransmit(HM5883_DATAX0 );
 res1= iicrestart(magnet_rd); 
 iicswrcv();
 
 for(i=0; i<4  ;i++) {
  buff[i]=iicreceive();
 }
 
 buff[i]= iicreceivem1();
 buff[i+1]= iicreceivelast();

	*mxraw = ((buff[0] << 8) | buff[1]);
	*myraw = ((buff[2] << 8) | buff[3]);
	*mzraw = ((buff[4] << 8) | buff[5]);
}  


void accel_init (void){
  
 int  res1;
 
 res1=iicstart(accel_wr);
 res1=iictransmit(ADXL345_POWER_CTL );  //  
 res1=iictransmit(0x08 );
  
 res1=iictransmit(ADXL345_DATA_FORMAT );  // ; 
 res1=iictransmit(0x08 );
  
 iicstop();  
}


void adxl345_getrawdata(int *axraw, int *ayraw, int *azraw){
  
 uint8_t i = 0;
 uint8_t buff[6];
 int res1;
	
 res1=iicstart(accel_wr);
 res1=iictransmit(ADXL345_DATAX0 );
 res1= iicrestart(accel_rd); 
 iicswrcv();
 
 for(i=0; i<4  ;i++) {
  buff[i]=iicreceive();
 }
 
 buff[i]= iicreceivem1();
 buff[i+1]= iicreceivelast();

	*axraw = ((buff[1] << 8) | buff[0]);
	*ayraw = ((buff[3] << 8) | buff[2]);
	*azraw = ((buff[5] << 8) | buff[4]);
}  
  

 //  Gyro Initialisation
 
 void gyro_init (void) {
  
 int  res1;
 
 res1=iicstart(gyro_wr);
 res1=iictransmit(L3G4200D_CTRL_REG1 );  // ; 100hz, 12.5Hz, Power up
 res1=iictransmit(0x0f );
 iicstop();  
 }
 
 
// Function to get a set of gyro data
// Eduardo Nebot,30 July 2015 

void l3g4200d_getrawdata(int *gxraw, int *gyraw, int *gzraw) {
 	uint8_t i = 0;
	uint8_t buff[6];
	int res1;
	
   res1=iicstart(gyro_wr);
   res1=iictransmit(L3G4200D_OUT_XYZ_CONT );
   res1= iicrestart(gyro_rd); 
 
 iicswrcv();
 
 for(i=0; i<4  ;i++) {
  buff[i]=iicreceive();
 }
 
buff[i]= iicreceivem1();
buff[i+1]= iicreceivelast();

	*gxraw = ((buff[1] << 8) | buff[0]);
	*gyraw = ((buff[3] << 8) | buff[2]);
	*gzraw = ((buff[5] << 8) | buff[4]);
}

// ********************