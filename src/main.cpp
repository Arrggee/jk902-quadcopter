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

#define SAMPLE_RATE 50
#define LED 13

L3G gyro;
LSM303 compass;
Madgwick filter;
IMUHelper helper(&gyro, &compass, &filter);

uint8_t ledState = HIGH;
unsigned long timeNow, timePrevLED, timePrev;

Reading reading;

void setup() {
    Serial.println("Begin setup");
    Serial.begin(9600);
    Wire.begin();

    Serial.println("Init IMU");
    gyro.init();
    gyro.enableDefault();
    compass.init();
    compass.enableDefault();
    Serial.println("IMU Initialised");

    pinMode(LED, OUTPUT);
    digitalWrite(LED, ledState);

    filter.begin(SAMPLE_RATE);
    Serial.println("Setup complete");

    timePrevLED = millis();
    timePrev = millis();
}

void loop() {
    timeNow = millis();
    if (timeNow >= timePrevLED + 1000) {
        ledState = ledState == HIGH ? LOW : HIGH;
        digitalWrite(LED, ledState);
        timePrevLED = timeNow;
    }

    if (timeNow > timePrev + SAMPLE_RATE) {
        helper.printOrientation();
        timePrev = timeNow;
    }
}
