#include "Servo.h"

// Connections
Servo s;

const int buttonPin = 7;
const int switchPin = 8;
const int servoPin = 9;

// Measurements
const double diameter = 9.0; // average diameter of BACK wheels in cm
const double circumference = PI*diameter;

////// COMP PARAMS //////
const double TARGET_DIST = 1000.0; // in cm
const double TARGET_REVS = TARGET_DIST / circumference;
const int TARGET_REVS_FLOOR = (int)TARGET_REVS;

const double TARGET_TIME = 20.0; // in s
const double TARGET_TIME_MS = TARGET_TIME*1000.0; // in ms

// Speed logic
const int maxServoSpeed = 0; // fastest the servo can run without the limit switch skipping revs
const double maxEndRealSpeed = 43.62688; // see spreadsheet & journal to see how I got this
const int maxEndServoSpeed = 72;

double dist = 0.0; // current distance from start line
double currentFinalEndDist = 0.0; // recording distance from the last press of the switch

const int endRevs = 1; // # of revs before end line to stop rolling, excluding any partial revs because TARGET_DIST does not divide with revs as a whole
const double bufferDist = endRevs * circumference; // distance from the beginning of the "buffer" revs to the last "buffer" rev - e.g. if endRevs = 1, then bufferDist = circumference, if eR = 2, bD = 2*circ, etc.
const double finalDist = (TARGET_REVS - TARGET_REVS_FLOOR) * circumference; // distance from last rev to end
const double endDist = bufferDist + finalDist; // how much dist is left before the end - endRevs revs + the rest

double endTime; // how much time is left before the end
double endRealSpeed; // calculated later with endDist/Time
double endServoSpeed; // this is what you put in

int currentSpeed = -1; // current speed

// Button properties & debounce
int revs = 0; // how many revs you've GONE

int lastButtonState = HIGH;
int lastSwitchState = HIGH;

int currentButtonState;
int currentSwitchState = HIGH; // default not pressed

unsigned long lastDebounceTime = 0;
const int debounceDelay = 5; // for limitswitch

// Flags
bool start = false;
bool triggerSlow = false;
bool slow = false;
bool triggerFinal = false;
bool calculateFinalEndDist = false;
bool end = false;

// Timing
double t; // in ms
double finalEndT; // millis() time at the start of the final period
double endElapsedTime;
double currentTime;
bool startTiming = false;
void updateTime() { currentTime = millis() - t; }


void setup() {
    Serial.begin(9600);
    Serial.println("Setup started");

    // Connections
    s.attach(servoPin);
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(switchPin, INPUT_PULLUP);
}

void loop() {
    // Start button
    currentButtonState = digitalRead(buttonPin);
    if (!start && currentButtonState == LOW && lastButtonState == HIGH) { // LOW = pressed, HIGH = not pressed
        start = true;
        startTiming = true;
    }
    lastButtonState = currentButtonState;
    
    // Main logic
    if (start) {
        // Initial commands after starting
        if (startTiming) {
            t = millis();

            currentSpeed = -maxServoSpeed;
            s.write(-maxServoSpeed); // negative for the right direction

            startTiming = false;
        }

        // Time update
        if (!end) {
            updateTime();
        }

        // Serial print time exceeded
        if (!end && currentTime >= TARGET_TIME_MS) { // If TARGET_TIME_MS exceeded
            Serial.println("TIME EXCEEDED BY: " + String((millis()-t-TARGET_TIME_MS)/1000.0) + " s");
        }

        // Limit switch
        int switchReading = digitalRead(switchPin); // temporary reading that will be assigned to currentSwitchState if confirmed
        if (switchReading != lastSwitchState) { lastDebounceTime = millis(); } // start debounce timer
        if ((millis() - lastDebounceTime) > debounceDelay) { // signal cleared debounce delay
            if (switchReading != currentSwitchState) { // confirm debounce
                currentSwitchState = switchReading;
                if (!calculateFinalEndDist && currentSwitchState == LOW) { // confirm LOW
                    revs++;
                    dist = revs*circumference; // recalculate
                    Serial.println("revs = " + String(revs));
                }
            }
        }
        lastSwitchState = switchReading;

        // Beginning of slow period / slow period trigger
        if (revs == TARGET_REVS_FLOOR - endRevs && !triggerSlow) {
            // Trigger flag to compute end parameters and slow down
            slow = true;
            triggerSlow = true;
        }

        // Slow period logic
        if (slow) {
            // Compute endTime, endSpeed
            endTime = (TARGET_TIME_MS - currentTime)/1000.0; // in s

            if (currentTime < TARGET_TIME_MS && abs(endTime) > endDist/maxEndRealSpeed) { // if the car has not passed the target time AND has adequate time to cross (meaning there is more time than the time it would take to run at maxEndSpeed)
                endRealSpeed = endDist/endTime;
                Serial.println(endRealSpeed);
                endServoSpeed = -0.389298 * endRealSpeed + 88.70013; // regression model from spreadsheet
            } else {
                // move at max possible reliable speed to get to finish line at endDist --> see journal and spreadsheet to see how I got this number
                endServoSpeed = maxEndServoSpeed;
                endRealSpeed = maxEndRealSpeed;
            }

            currentSpeed = endServoSpeed;
            s.write(endServoSpeed);
            Serial.println("current speed = " + String(currentSpeed));

            slow = false;
        }

        // Beginning of final period, i.e. period beginning after switch is hit for the last time / final period trigger
        if (revs == TARGET_REVS_FLOOR && !triggerFinal) {
            calculateFinalEndDist = true;
            finalEndT = millis();
            triggerFinal = true;
        }

        // Final period logic
        if (calculateFinalEndDist) {
            endElapsedTime = millis() - finalEndT;
            currentFinalEndDist = endRealSpeed * (endElapsedTime / 1000.0); // also convert endElapsedTime to secs
        }

        // Beginning of end period / end period trigger
        if (currentFinalEndDist >= finalDist) {
            dist += currentFinalEndDist;
            
            calculateFinalEndDist = false;
            end = true;
        }

        // End period logic - you're done!
        if (end) {
            currentSpeed = 90;
            s.write(90);

            // End stats
            Serial.println("Total distance covered: " + String(dist));
            Serial.println("Total time elapsed: " + String(currentTime));

            start = false;
        }
    }

    // Serial.println("current speed = " + String(currentSpeed));
    // Serial.println("revs = " + String(revs));
}