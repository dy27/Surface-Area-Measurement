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