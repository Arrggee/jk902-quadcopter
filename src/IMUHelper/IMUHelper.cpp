#include "IMUHelper.h"

void IMUHelper::getCalibratedAcc(Reading *reading)
{
    compass->read();
    reading->acc_x = compass->a.x * ACC_CALIB;
    reading->acc_y = compass->a.y * ACC_CALIB;
    reading->acc_z = compass->a.z * ACC_CALIB;
}

void IMUHelper::getCalibratedMag(Reading *reading)
{
    compass->read();
    reading->mag_x = (compass->m.x - MAG_OFF_X) * MAG_SCA_X * MAG_CALIB;
    reading->mag_y = (compass->m.y - MAG_OFF_Y) * MAG_SCA_Y * MAG_CALIB;
    reading->mag_z = (compass->m.z - MAG_OFF_Z) * MAG_SCA_Z * MAG_CALIB;
}

void IMUHelper::getCalibratedGyro(Reading *reading)
{
    gyro->read();
    reading->gyro_x = -gyro->g.y * GYRO_CALIB;
    reading->gyro_y = gyro->g.x * GYRO_CALIB;
    reading->gyro_z = gyro->g.z * GYRO_CALIB;
}

void IMUHelper::getReading(Reading *reading)
{
    getCalibratedAcc(reading);
    getCalibratedMag(reading);
    getCalibratedGyro(reading);
}
