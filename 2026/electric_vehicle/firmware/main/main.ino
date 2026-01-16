#include "Servo.h"

////// COMP PARAMS //////
const double TARGET_DIST = 800.0; // in cm
const double TARGET_TIME = 15.0; // in s
const double calibrationCoefficient = 0.0;
//////             //////

const double d = 9.0; // average diameter of FRONT wheels in cm
const double ratio = 24.0/22.5; // 24mm (servo gear) to 22.5mm (shaft gear) ratio

const bool dir = true; // true = facing front

const double speed = TARGET_DIST/TARGET_TIME;
const double rpm = 180.0;
const double wheelRPM = rpm*ratio;

const double maxSpeed = wheelRPM*2.0*PI*(d/2.0)*(1.0/60.0); // in cm/s
double servoSpeed = map(speed, 0.0, maxSpeed, 90.0, 180.0); // 100% speed = max wheel RPM = wheelRPM*2*pi*(d/2)*(1/60), include calibration constant

Servo s;

const int servoPin = 9;

void setup() {
    Serial.begin(9600);
    Serial.println("Setup started");

    s.attach(servoPin);
    
    delay(5000); // testing delay

    Serial.println((int)speed + " cm/s");
    // Hold servoSpeed for TARGET_TIME
    if (dir) { servoSpeed *= -1; }
    s.write((int)servoSpeed);
    delay(TARGET_TIME * 1000.0);
    s.write(90); // stop
}
void loop() {
    Serial.println((int)speed + " cm/s");
}