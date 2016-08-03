#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>
#include <math.h>

#ifndef IMU_HELPER
#define IMUHELPER

#define ACC_CALIB (0.00005f)
//#define ACC_CALIB (1)
#define GYRO_CALIB (0.01f)
//#define GYRO_CALIB (1)
#define MAG_CALIB (0.0001f)
//#define MAG_CALIB (1)

#define MAG_OFF_X -153
#define MAG_OFF_Y -798
#define MAG_OFF_Z 226
#define MAG_SCA_X 0.9611f
#define MAG_SCA_Y 0.9524f
#define MAG_SCA_Z 1.023f

struct Reading {
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
};

class IMUHelper
{
public:
    void getReading(Reading *reading);
    IMUHelper(L3G *gyro_in, LSM303 *compass_in) :
        gyro(gyro_in),
        compass(compass_in)
    {}
private:
    L3G *gyro;
    LSM303 *compass;

    void getCalibratedAcc(Reading *reading);
    void getCalibratedGyro(Reading *reading);
    void getCalibratedMag(Reading *reading);
};

#endif