#include "IMUHelper.h"

void IMUHelper::getCalibratedAcc(Reading *reading) {
    compass->read();
    reading->acc_x = compass->a.x * ACC_CALIB;
    reading->acc_y = compass->a.y * ACC_CALIB;
    reading->acc_z = compass->a.z * ACC_CALIB;
}

void IMUHelper::getCalibratedMag(Reading *reading) {
    compass->read();
    reading->mag_x = (compass->m.x - MAG_OFF_X) * MAG_SCA_X * MAG_CALIB;
    reading->mag_y = (compass->m.y - MAG_OFF_Y) * MAG_SCA_Y * MAG_CALIB;
    reading->mag_z = (compass->m.z - MAG_OFF_Z) * MAG_SCA_Z * MAG_CALIB;
}

void IMUHelper::getCalibratedGyro(Reading *reading) {
    gyro->read();
    reading->gyro_x = -gyro->g.y * GYRO_CALIB;
    reading->gyro_y = gyro->g.x * GYRO_CALIB;
    reading->gyro_z = gyro->g.z * GYRO_CALIB;
}

void IMUHelper::getReading(Reading *reading) {
    getCalibratedAcc(reading);
    getCalibratedMag(reading);
    getCalibratedGyro(reading);
}

void IMUHelper::printReading(Reading *r) {
    getReading(r);
    Serial.print(r->acc_x);Serial.print(",");
    Serial.print(r->acc_y);Serial.print(",");
    Serial.print(r->acc_z);Serial.print(",");
    Serial.print(r->gyro_x);Serial.print(",");
    Serial.print(r->gyro_y);Serial.print(",");
    Serial.print(r->gyro_z);Serial.print(",");
    Serial.print(r->mag_x);Serial.print(",");
    Serial.print(r->mag_y);Serial.print(",");
    Serial.print(r->mag_z);Serial.print(",");
    Serial.println(millis());
}

void IMUHelper::printReading(void) {
    printReading(&_reading);
}

void IMUHelper::printOrientation(void) {
    getReading(&_reading);
    filter->update(
            _reading.gyro_x, _reading.gyro_y, _reading.gyro_z,
            _reading.acc_x, _reading.acc_y, _reading.acc_z,
            _reading.mag_x, _reading.mag_y, _reading.mag_z
    );

    roll = filter->getRoll();
    pitch = filter->getPitch();
    heading = filter->getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
}

void IMUHelper::getOrientation(Orientation *o) {
    getReading(&_reading);
    filter->update(
            _reading.gyro_x, _reading.gyro_y, _reading.gyro_z,
            _reading.acc_x, _reading.acc_y, _reading.acc_z,
            _reading.mag_x, _reading.mag_y, _reading.mag_z
    );
    o->pitch = filter->getPitch();
    o->roll = filter->getRoll();
    o->yaw = filter->getYaw();
}
