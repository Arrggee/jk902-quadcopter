/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low gyro data registers.
They can be converted to units of dps (degrees per second) using the
conversion factors specified in the datasheet for your particular
device and full scale setting (gain).
Example: An L3GD20H gives a gyro X axis reading of 345 with its
default full scale setting of +/- 245 dps. The So specification
in the L3GD20H datasheet (page 10) states a conversion factor of 8.75
mdps/LSB (least significant bit) at this FS setting, so the raw
reading of 345 corresponds to 345 * 8.75 = 3020 mdps = 3.02 dps.
*/

#include "Arduino.h"
#include "Wire.h"
#include "L3G.h"
#include "LSM303.h"
#include "IMUHelper/IMUHelper.h"
#include "MadgwickAHRS.h"

#define SAMPLE_RATE 50

L3G gyro;
LSM303 compass;
Reading r;
IMUHelper helper(&gyro, &compass);
Madgwick filter;

char report[80];
unsigned long timeNow, timePrev;
float roll, pitch, heading;

void printAll() {

    helper.getReading(&r);
    Serial.print(r.acc_x);Serial.print(",");
    Serial.print(r.acc_y);Serial.print(",");
    Serial.print(r.acc_z);Serial.print(",");
    Serial.print(r.gyro_x);Serial.print(",");
    Serial.print(r.gyro_y);Serial.print(",");
    Serial.print(r.gyro_z);Serial.print(",");
    Serial.print(r.mag_x);Serial.print(",");
    Serial.print(r.mag_y);Serial.print(",");
    Serial.print(r.mag_z);Serial.print(",");
    Serial.println(timeNow);
}

void printOrientation() {
    helper.getReading(&r);
    filter.update(
        r.gyro_x, r.gyro_y, r.gyro_z,
        r.acc_x, r.acc_y, r.acc_z,
        r.mag_x, r.mag_y, r.mag_z
    );

    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
}

void setup() {
    Serial.begin(9600);
    Wire.begin();

    gyro.init();
    gyro.enableDefault();
    compass.init();
    compass.enableDefault();

    filter.begin(SAMPLE_RATE);

    timePrev = millis();
}

void loop() {
    timeNow = millis();
    if (timeNow >= timePrev + SAMPLE_RATE) {
        printOrientation();
        timePrev = timeNow;
    }
}
