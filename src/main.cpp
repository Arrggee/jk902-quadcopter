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
#include "PID_v1.h"
#include <Servo.h>
#include "IMUHelper/IMUHelper.h"

#define SAMPLE_RATE 50
#define LED 13

#define BASE_THROTTLE 1500

#define MIN_THROTTLE 1000
#define MAX_THROTTLE 2000

// LEDs, Timers
uint8_t ledState = HIGH;
unsigned long timeNow, timePrevLED, timePrev;

// Sensors
L3G gyro;
LSM303 compass;
Madgwick filter;
IMUHelper helper(&gyro, &compass, &filter);

Reading reading;
Orientation o;

// PIDs
double pidRollOut, pidPitchOut;
double setPoint = -0.5;
double consKp=3, consKi=0, consKd=0.5;

PID pitchPid(&(o.pitch), &pidPitchOut, &setPoint, consKp, consKi, consKd, DIRECT);
PID rollPid(&(o.roll), &pidRollOut, &setPoint, consKp, consKi, consKd, DIRECT);

// Motors
Servo fl;
Servo fr;
Servo bl;
Servo br;

void spinDown() {
    int throttle = 1250;
    for (;throttle >= 1000; throttle -= 50) {
        fl.writeMicroseconds(throttle);
        fr.writeMicroseconds(throttle);
        bl.writeMicroseconds(throttle);
        br.writeMicroseconds(throttle);
        delay(200);
    }
    for(;;);
}

void setup() {
    Serial.begin(9600);
    Wire.begin();

    fl.attach(6);
    fr.attach(7);
    bl.attach(8);
    br.attach(9);
    fl.writeMicroseconds(MIN_THROTTLE);
    fr.writeMicroseconds(MIN_THROTTLE);
    bl.writeMicroseconds(MIN_THROTTLE);
    br.writeMicroseconds(MIN_THROTTLE);

    pitchPid.SetMode(AUTOMATIC);
    pitchPid.SetOutputLimits(-100, 100);
    rollPid.SetMode(AUTOMATIC);
    pitchPid.SetOutputLimits(-100, 100);

    gyro.init();
    gyro.enableDefault();
    compass.init();
    compass.enableDefault();
    filter.begin(SAMPLE_RATE);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, ledState);

    timePrevLED = millis();
    timePrev = millis();
}

void loop() {
    timeNow = millis();

    // LED Blink
    if (timeNow >= timePrevLED + 1000) {
        ledState = (uint8_t) (ledState == HIGH ? LOW : HIGH);
        digitalWrite(LED, ledState);
        timePrevLED = timeNow;
    }

    if (timeNow > timePrev + SAMPLE_RATE) {
        helper.getReading(&reading);
        timePrev = timeNow;

        if (timeNow > 10000 && timeNow < 40000) {
            helper.getOrientation(&o);
            Serial.print("Pitch: ");
            Serial.println(*(&(o.pitch)));
            Serial.print("PID out: ");
            Serial.println(pidPitchOut);
            pitchPid.Compute();
            rollPid.Compute();
            fl.writeMicroseconds((int) (BASE_THROTTLE - pidPitchOut - pidRollOut));
            fr.writeMicroseconds((int) (BASE_THROTTLE - pidPitchOut + pidRollOut));
            bl.writeMicroseconds((int) (BASE_THROTTLE + pidPitchOut - pidRollOut));
            br.writeMicroseconds((int) (BASE_THROTTLE + pidPitchOut + pidRollOut));
        }
        if (timeNow > 40000) {
            spinDown();
            fl.writeMicroseconds(MIN_THROTTLE);
            fr.writeMicroseconds(MIN_THROTTLE);
            bl.writeMicroseconds(MIN_THROTTLE);
            br.writeMicroseconds(MIN_THROTTLE);
        }
    }
}
