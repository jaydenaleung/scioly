////// COMP PARAMS //////
const double TARGET_DIST = 0.0; // in cm
const double TARGET_TIME = 0.0; // in s
const double calibrationCoefficient = 0.0;
//////             //////

const double d = 9.0; // average diameter of FRONT wheels in cm
const double speed = TARGET_DIST/TARGET_TIME;
const double maxSpeed = 230*2*PI*(d/2)*(1/60); // in cm/s
const double servoSpeed = map(speed, 0, maxSpeed, 90, 180); // 100% speed = 230 RPM = 230*2*pi*(d/2)*(1/60), include calibration constant

Servo s;
const int servoPin = 10;

void setup() {
    s.attach(servoPin);
}
void loop() {
    // Hold servoSpeed for TARGET_TIME
    s.write(servoSpeed);
    delay(TARGET_TIME);
}