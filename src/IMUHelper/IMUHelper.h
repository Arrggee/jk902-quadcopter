#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>
#include <math.h>
#include <MadgwickAHRS.h>

#ifndef IMU_HELPER
#define IMU_HELPER

#define ACC_CALIB (0.000061f)
#define GYRO_CALIB (0.00875f)
#define MAG_CALIB (0.00016f)

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

struct Orientation {
    double pitch, roll, yaw;
};

class IMUHelper {
public:
    void getReading(Reading *reading);
    void printReading(void);
    void printReading(Reading *reading);
    void getOrientation(Orientation *o);
    void printOrientation(void);
    IMUHelper(L3G *gyro_in, LSM303 *compass_in, Madgwick *filter_in) :
        gyro(gyro_in),
        compass(compass_in),
        filter(filter_in)
    {}
private:
    L3G *gyro;
    LSM303 *compass;
    Madgwick *filter;
    Reading _reading;

    float pitch, roll, heading;

    void getCalibratedAcc(Reading *reading);
    void getCalibratedGyro(Reading *reading);
    void getCalibratedMag(Reading *reading);
};

#endif