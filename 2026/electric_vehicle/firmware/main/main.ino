/*
 * =============================================================================
 *  Science Olympiad Electric Vehicle — Cascaded PID Controller
 *  Arduino Uno R3
 * =============================================================================
 *
 *  OVERVIEW
 *  --------
 *  This firmware drives a two-wheeled robot along a trapezoidal track whose
 *  geometry is fully computed from two day-of competition parameters:
 *      TARGET_DIST_CM  – straight-line distance from start to finish (cm)
 *      TARGET_TIME_S   – total time allowed to traverse the track (s)
 *
 *  A cascaded PID architecture is used:
 *      1. Positional PID (outer loop) — uses goBILDA 4-bar odometry pods
 *         to obtain the robot's (x, y, heading) pose and produces a desired
 *         forward velocity and a heading-correction term.
 *      2. Velocity PIDs (inner loops, one per wheel) — each reads a
 *         600 PP/R quadrature encoder, computes the current wheel speed, and
 *         drives a goBILDA continuous-rotation servo to match the velocity
 *         set-point produced by the outer loop.
 *
 *  MEASUREMENT-POINT OFFSET
 *  ------------------------
 *  The competition measurement point is NOT at the wheel-axle midpoint.
 *  In the robot body frame (origin = midpoint between wheel centres,
 *  +Y = forward, +X = right) the measurement point is at (0, +60 mm).
 *  All waypoint tracking is done relative to this offset point, so
 *  odometry readings are transformed before the positional PID runs.
 *
 *  GEAR TRAIN
 *  ----------
 *  Wheel <-- 45T <--> 60T (servo) <--> 83T --> Encoder
 *  - Wheel and encoder shafts spin the SAME direction.
 *  - Servo shaft spins OPPOSITE to both.
 *  - Wheel speed   = servo speed * (60/45)
 *  - Encoder speed = servo speed * (60/83)
 *  - Wheel-to-encoder ratio = 83/45
 *
 *  TRACK GEOMETRY  (see problem statement diagram)
 *  -----------------------------------------------
 *  Start --D1--> Turn A1 --D2--> Turn 90 --D3--> Turn 90 --D2-->
 *          Turn A2 --D4--> Finish
 *
 *  D2 = 150 mm  (fixed)
 *  D3 = 2000 mm (fixed, middle leg)
 *  Can perpendicular offset = 1000 mm
 *  CAN_SPACING ~ 300 mm (user-adjustable)
 *  D1, D4, A1, A2 computed from TARGET_DIST and the above.
 *
 *  FINAL-APPROACH STRATEGY
 *  -----------------------
 *  When the robot is within SLOW_DIST_MM of the finish waypoint:
 *      (a) If remaining time > distance / current speed -> slow down so the
 *          measurement point arrives at the finish in exactly the remaining time.
 *      (b) Otherwise -> keep current speed (already behind schedule).
 *
 *  HARDWARE CONNECTIONS (Arduino Uno)
 *  ----------------------------------
 *      Pin  2  – Left  encoder channel A  (interrupt 0)
 *      Pin  3  – Right encoder channel A  (interrupt 1)
 *      Pin  4  – Left  encoder channel B
 *      Pin  5  – Right encoder channel B
 *      Pin  7  – Start button (INPUT_PULLUP, active LOW)
 *      Pin  9  – Left  servo PWM
 *      Pin 10  – Right servo PWM
 *      I2C (A4/A5) – goBILDA Odometry Computer
 *
 * =============================================================================
 */

#include <Servo.h>
#include <Wire.h>

/* ─────────────────────────────────────────────────────────────────────────────
 *  goBILDA Odometry Computer I2C Interface
 *  The goBILDA odometry computer communicates over I2C and provides
 *  x (mm), y (mm), and heading (radians) of the robot.
 * ───────────────────────────────────────────────────────────────────────────── */
#define ODOM_I2C_ADDR  0x30        // Default I2C address for goBILDA odom computer

// Register map (from goBILDA documentation)
#define REG_X_POS      0x04        // X position, 2 bytes, signed, mm
#define REG_Y_POS      0x06        // Y position, 2 bytes, signed, mm
#define REG_HEADING    0x08        // Heading,    2 bytes, signed, 0.01 rad units

/* ─────────────────────────────────────────────────────────────────────────────
 *  USER-ADJUSTABLE / DAY-OF COMPETITION PARAMETERS
 *  >>>>> SET THESE ON COMPETITION DAY <<<<<
 * ───────────────────────────────────────────────────────────────────────────── */
double TARGET_DIST_CM = 810.0;     // Straight-line start-to-finish distance (cm)
double TARGET_TIME_S  = 15.0;      // Allowed traverse time (seconds, 10-20)

// Track geometry tweaks (mm)
const double CAN_SPACING_MM   = 300.0;   // Distance between the two cans (mm)
const double CAN_PERP_MM      = 1000.0;  // Perpendicular offset of farthest can (mm)
const double D2_MM             = 150.0;   // Short connector legs (mm)
const double D3_MM             = 2000.0;  // Middle leg length (mm)

// Final-approach / slow-down region (mm) — user sets between 0 and 1000
double SLOW_DIST_MM = 500.0;

/* ─────────────────────────────────────────────────────────────────────────────
 *  PHYSICAL CONSTANTS
 * ───────────────────────────────────────────────────────────────────────────── */

// Wheel & chassis
const double WHEEL_DIAMETER_MM     = 90.0;
const double WHEEL_CIRCUMFERENCE   = PI * WHEEL_DIAMETER_MM;          // mm
const double WHEEL_BASE_MM         = 107.194;                         // centre-to-centre

// Measurement-point offset from wheel-axle midpoint (body frame, mm)
// In body frame: +Y = forward, +X = right, origin = midpoint of wheels
const double MEAS_POINT_X          = 0.0;    // centered between wheels
const double MEAS_POINT_Y          = 60.0;   // 60 mm forward of axle midpoint

// Gear teeth
const double GEAR_SERVO   = 60.0;
const double GEAR_WHEEL   = 45.0;
const double GEAR_ENCODER = 83.0;

// Derived gear ratios
const double RATIO_ENCODER_TO_WHEEL = GEAR_ENCODER / GEAR_WHEEL;  // 1.8444

// Encoder
const int    ENCODER_PPR = 600;  // pulses per revolution of the encoder shaft
// Distance per encoder pulse at the wheel
const double MM_PER_ENCODER_PULSE = WHEEL_CIRCUMFERENCE * RATIO_ENCODER_TO_WHEEL
                                    / (double)ENCODER_PPR;

/* ─────────────────────────────────────────────────────────────────────────────
 *  PIN DEFINITIONS
 * ───────────────────────────────────────────────────────────────────────────── */
const int PIN_ENC_L_A   = 2;    // Left encoder channel A  (INT0)
const int PIN_ENC_R_A   = 3;    // Right encoder channel A (INT1)
const int PIN_ENC_L_B   = 4;    // Left encoder channel B
const int PIN_ENC_R_B   = 5;    // Right encoder channel B
const int PIN_BUTTON    = 7;    // Start button
const int PIN_SERVO_L   = 9;    // Left servo
const int PIN_SERVO_R   = 10;   // Right servo

/* ─────────────────────────────────────────────────────────────────────────────
 *  SERVO OBJECTS
 * ───────────────────────────────────────────────────────────────────────────── */
Servo servoL;
Servo servoR;

// Continuous-rotation servo: 1500 us = stop
const int SERVO_STOP = 1500;

/* ─────────────────────────────────────────────────────────────────────────────
 *  ENCODER STATE  (volatile — updated in ISRs)
 * ───────────────────────────────────────────────────────────────────────────── */
volatile long encCountL = 0;
volatile long encCountR = 0;

// ISR for left encoder channel A — reads channel B for direction
void isrLeftEncoder() {
    if (digitalRead(PIN_ENC_L_B) == HIGH) encCountL++;
    else                                   encCountL--;
}

// ISR for right encoder channel A — reads channel B for direction
void isrRightEncoder() {
    if (digitalRead(PIN_ENC_R_B) == HIGH) encCountR++;
    else                                   encCountR--;
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  goBILDA ODOMETRY HELPERS
 * ───────────────────────────────────────────────────────────────────────────── */

// Read a signed 16-bit register pair from the odometry computer over I2C
int16_t odomRead16(uint8_t reg) {
    Wire.beginTransmission(ODOM_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)ODOM_I2C_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return 0;
    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    return (int16_t)((hi << 8) | lo);
}

// Robot pose from odometry computer (all in mm / radians)
double odomX       = 0.0;   // mm, field frame
double odomY       = 0.0;   // mm, field frame
double odomHeading = 0.0;   // radians

// Read latest pose from the goBILDA odometry computer
void updateOdometry() {
    odomX       = (double)odomRead16(REG_X_POS);           // mm
    odomY       = (double)odomRead16(REG_Y_POS);           // mm
    odomHeading = (double)odomRead16(REG_HEADING) * 0.01;  // centirads to rads
}

/*
 * Transform odometry (reports the robot centre / axle midpoint)
 * to the measurement point, which is 60 mm forward in the body frame.
 *
 *   measX = odomX + MEAS_POINT_Y * sin(heading)
 *   measY = odomY + MEAS_POINT_Y * cos(heading)
 *
 * (MEAS_POINT_X is 0 so the lateral term drops out.)
 */
double measX = 0.0;
double measY = 0.0;

void computeMeasurementPoint() {
    measX = odomX + MEAS_POINT_Y * sin(odomHeading);
    measY = odomY + MEAS_POINT_Y * cos(odomHeading);
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  TRACK GEOMETRY COMPUTATION
 *
 *  Coordinate system (field frame, mm):
 *      Origin  = starting point (where measurement point begins)
 *      +Y      = direction the robot initially faces (forward / perpendicular)
 *      +X      = to the right (toward finish from start)
 *
 *  The start and finish are on the same horizontal line, separated by
 *  TARGET_DIST.  The robot drives forward (perpendicular), turns,
 *  traverses the middle section past the cans, turns, and comes back
 *  down to the finish.
 *
 *  Waypoints (6 total):
 *      WP0 = Start          (0, 0)
 *      WP1 = First turn     at end of D1 leg
 *      WP2 = Start of D3    (after first D2 connector)
 *      WP3 = End of D3      (start of second D2 connector)
 *      WP4 = Last turn      at end of second D2 connector
 *      WP5 = Finish         (TARGET_DIST_MM, 0)
 * ───────────────────────────────────────────────────────────────────────────── */

// Computed track values (set in computeTrack())
double D1_MM = 0.0;
double D4_MM = 0.0;
double A1_RAD = 0.0;
double A2_RAD = 0.0;

// Waypoints in field frame (mm)
#define NUM_WAYPOINTS 6
double wpX[NUM_WAYPOINTS];
double wpY[NUM_WAYPOINTS];

/*
 * computeTrack()
 *
 * Computes the full track geometry from TARGET_DIST_CM.
 *
 * The middle segment (D3) runs horizontally at Y = CAN_PERP_MM.
 * D2 connectors are vertical, length 150 mm.
 * D1 and D4 are the diagonal legs from start/finish up to the connectors.
 *
 *   x2 = TARGET_DIST_MM/2 - D3_MM/2    (X of WP2)
 *   yLeg = CAN_PERP_MM - D2_MM         (Y of WP1 and WP4)
 *
 *   D1 = sqrt(x2^2 + yLeg^2)
 *   D4 = sqrt((TARGET_DIST_MM - (x2+D3_MM))^2 + yLeg^2)
 *   A1 = atan2(x2, yLeg)
 *   A2 = atan2(TARGET_DIST_MM - (x2+D3_MM), yLeg)
 */
void computeTrack() {
    double targetDistMM = TARGET_DIST_CM * 10.0;   // cm to mm

    double halfTarget = targetDistMM / 2.0;
    double halfD3     = D3_MM / 2.0;

    // X-coordinate of WP2 (start of middle segment)
    double x2 = halfTarget - halfD3;

    // Perpendicular (Y) distance from start/finish line to WP1/WP4
    double yLeg = CAN_PERP_MM - D2_MM;   // 1000 - 150 = 850 mm

    // Compute D1 and D4 (hypotenuse / diagonal legs)
    double dx1 = x2;
    D1_MM = sqrt(dx1 * dx1 + yLeg * yLeg);

    double dx4 = targetDistMM - (x2 + D3_MM);
    D4_MM = sqrt(dx4 * dx4 + yLeg * yLeg);

    // Compute turn angles (radians)
    A1_RAD = atan2(dx1, yLeg);
    A2_RAD = atan2(dx4, yLeg);

    // Build waypoint list
    // WP0: Start
    wpX[0] = 0.0;
    wpY[0] = 0.0;

    // WP1: End of D1 / bottom of first D2 connector
    wpX[1] = x2;
    wpY[1] = yLeg;

    // WP2: Top of first D2 connector / start of D3
    wpX[2] = x2;
    wpY[2] = CAN_PERP_MM;

    // WP3: End of D3 / top of second D2 connector
    wpX[3] = x2 + D3_MM;
    wpY[3] = CAN_PERP_MM;

    // WP4: Bottom of second D2 connector / start of D4
    wpX[4] = x2 + D3_MM;
    wpY[4] = yLeg;

    // WP5: Finish
    wpX[5] = targetDistMM;
    wpY[5] = 0.0;

    // Print computed values for debugging
    Serial.println(F("=== TRACK COMPUTED ==="));
    Serial.print(F("D1 = ")); Serial.print(D1_MM); Serial.println(F(" mm"));
    Serial.print(F("D4 = ")); Serial.print(D4_MM); Serial.println(F(" mm"));
    Serial.print(F("A1 = ")); Serial.print(degrees(A1_RAD)); Serial.println(F(" deg"));
    Serial.print(F("A2 = ")); Serial.print(degrees(A2_RAD)); Serial.println(F(" deg"));
    for (int i = 0; i < NUM_WAYPOINTS; i++) {
        Serial.print(F("WP")); Serial.print(i);
        Serial.print(F(": (")); Serial.print(wpX[i]);
        Serial.print(F(", "));  Serial.print(wpY[i]);
        Serial.println(F(")"));
    }
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  PID CONTROLLER — generic struct, used for heading and velocity loops
 * ───────────────────────────────────────────────────────────────────────────── */
struct PIDController {
    double kp, ki, kd;
    double integral;
    double prevError;
    double outputMin, outputMax;
    unsigned long lastTime;

    // Initialise the PID with gains and output limits
    void init(double _kp, double _ki, double _kd, double _min, double _max) {
        kp = _kp;  ki = _ki;  kd = _kd;
        integral = 0.0;
        prevError = 0.0;
        outputMin = _min;
        outputMax = _max;
        lastTime = micros();
    }

    // Compute PID output given a setpoint and current measurement
    double compute(double setpoint, double measurement) {
        unsigned long now = micros();
        double dt = (double)(now - lastTime) / 1e6;   // seconds
        if (dt <= 0.0) dt = 0.001;                     // safety guard
        lastTime = now;

        double error = setpoint - measurement;

        // Proportional term
        double P = kp * error;

        // Integral term with anti-windup clamping
        integral += error * dt;
        double I = ki * integral;

        // Derivative term (on error)
        double derivative = (error - prevError) / dt;
        double D = kd * derivative;
        prevError = error;

        // Sum and clamp output
        double output = P + I + D;
        if (output > outputMax) { output = outputMax; integral -= error * dt; }
        if (output < outputMin) { output = outputMin; integral -= error * dt; }

        return output;
    }

    // Reset internal state (call when switching modes)
    void reset() {
        integral = 0.0;
        prevError = 0.0;
        lastTime = micros();
    }
};

/* ─────────────────────────────────────────────────────────────────────────────
 *  PID INSTANCES
 *
 *  Outer loop:  headingPID — error = desired heading - current heading
 *  Inner loops: velPID_L, velPID_R — one per wheel
 *
 *  Tuning parameters are starting points; adjust on the real robot.
 * ───────────────────────────────────────────────────────────────────────────── */
PIDController headingPID;
PIDController velPID_L;
PIDController velPID_R;

// Heading PID tuning: input = rad, output = mm/s differential correction
const double HDG_KP = 150.0;
const double HDG_KI =   5.0;
const double HDG_KD =  10.0;

// Velocity PID tuning: input = mm/s error, output = servo microseconds offset
const double VEL_KP = 0.8;
const double VEL_KI = 2.0;
const double VEL_KD = 0.01;

/* ─────────────────────────────────────────────────────────────────────────────
 *  SPEED TARGETS  (mm/s at the wheel)
 * ───────────────────────────────────────────────────────────────────────────── */
const double CRUISE_SPEED_MM_S    = 400.0;   // Straight-line cruise (fast but controllable)
const double TURN_SPEED_MM_S      = 150.0;   // Turning speed (slower for accuracy)
const double MIN_FINAL_SPEED_MM_S =  80.0;   // Minimum speed during final approach

/* ─────────────────────────────────────────────────────────────────────────────
 *  VELOCITY MEASUREMENT  (from wheel encoders)
 * ───────────────────────────────────────────────────────────────────────────── */
long prevEncL = 0, prevEncR = 0;
unsigned long velLastTime = 0;
double wheelVelL = 0.0;   // mm/s, left wheel
double wheelVelR = 0.0;   // mm/s, right wheel

// Compute wheel velocities from encoder counts
void updateWheelVelocities() {
    unsigned long now = micros();
    double dt = (double)(now - velLastTime) / 1e6;
    if (dt < 0.005) return;  // don't update faster than every 5 ms
    velLastTime = now;

    // Read encoder counts atomically
    noInterrupts();
    long cL = encCountL;
    long cR = encCountR;
    interrupts();

    long deltaL = cL - prevEncL;
    long deltaR = cR - prevEncR;
    prevEncL = cL;
    prevEncR = cR;

    // Convert encoder pulses to wheel mm/s
    double distL = (double)deltaL * MM_PER_ENCODER_PULSE;
    double distR = (double)deltaR * MM_PER_ENCODER_PULSE;

    wheelVelL = distL / dt;
    wheelVelR = distR / dt;
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  SERVO OUTPUT
 *
 *  Continuous-rotation servos:  1500 us = stop
 *      >1500 = one direction,  <1500 = other direction
 *
 *  Because the servo gear meshes between wheel and encoder gears, the
 *  servo shaft spins OPPOSITE to the wheel.  So to drive wheels forward
 *  we command the servo in the reverse direction.
 *
 *  Left servo:  forward wheel motion = servo us < 1500
 *  Right servo: forward wheel motion = servo us > 1500 (mirrored mounting)
 * ───────────────────────────────────────────────────────────────────────────── */
void setServoOutput(double pidOutL, double pidOutR) {
    // Positive PID output = forward wheel motion
    // Left: invert for gear reversal; Right: mirrored + gear reversal
    int usL = 1500 - (int)pidOutL;
    int usR = 1500 + (int)pidOutR;

    // Clamp to valid servo pulse range
    usL = constrain(usL, 1000, 2000);
    usR = constrain(usR, 1000, 2000);

    servoL.writeMicroseconds(usL);
    servoR.writeMicroseconds(usR);
}

void stopMotors() {
    servoL.writeMicroseconds(1500);
    servoR.writeMicroseconds(1500);
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  NAVIGATION STATE MACHINE
 * ───────────────────────────────────────────────────────────────────────────── */
enum NavState {
    NAV_IDLE,           // Waiting for start button
    NAV_STRAIGHT,       // Driving straight toward next waypoint
    NAV_TURNING,        // In-place turn at a waypoint
    NAV_FINAL_APPROACH, // Within SLOW_DIST_MM of final waypoint
    NAV_DONE            // Arrived at finish
};

NavState navState = NAV_IDLE;
int currentWP = 1;        // Index of waypoint we are driving TOWARD (start at 1)
double desiredHeading = 0.0;

const double WP_ARRIVE_THRESH = 25.0;   // Waypoint arrival threshold (mm)
const double TURN_THRESH       = 0.03;  // Turn completion threshold (~1.7 deg)

unsigned long startTimeMs = 0;           // millis() at run start

/* ─────────────────────────────────────────────────────────────────────────────
 *  HELPERS
 * ───────────────────────────────────────────────────────────────────────────── */

// Wrap angle to [-PI, PI]
double wrapAngle(double a) {
    while (a >  PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
}

// Euclidean distance between two 2D points
double dist2D(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

// Heading from (fromX,fromY) to (toX,toY), using atan2(dx,dy) with +Y forward
double headingTo(double fromX, double fromY, double toX, double toY) {
    return atan2(toX - fromX, toY - fromY);
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  SETUP
 * ───────────────────────────────────────────────────────────────────────────── */
void setup() {
    Serial.begin(9600);
    Serial.println(F("=== Electric Vehicle - Cascaded PID ==="));

    // I2C for goBILDA odometry computer
    Wire.begin();

    // Encoder pins
    pinMode(PIN_ENC_L_A, INPUT_PULLUP);
    pinMode(PIN_ENC_L_B, INPUT_PULLUP);
    pinMode(PIN_ENC_R_A, INPUT_PULLUP);
    pinMode(PIN_ENC_R_B, INPUT_PULLUP);

    // Attach interrupts for encoders (RISING edge on channel A)
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeftEncoder,  RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRightEncoder, RISING);

    // Start button
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    // Servos
    servoL.attach(PIN_SERVO_L);
    servoR.attach(PIN_SERVO_R);
    stopMotors();

    // Initialise PID controllers
    headingPID.init(HDG_KP, HDG_KI, HDG_KD, -200.0, 200.0);
    velPID_L.init(VEL_KP, VEL_KI, VEL_KD, -500.0, 500.0);
    velPID_R.init(VEL_KP, VEL_KI, VEL_KD, -500.0, 500.0);

    // Compute track geometry from competition parameters
    computeTrack();

    // Initial desired heading (toward WP1)
    desiredHeading = headingTo(wpX[0], wpY[0], wpX[1], wpY[1]);

    Serial.println(F("Setup complete. Press button to start."));
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  MAIN LOOP
 * ───────────────────────────────────────────────────────────────────────────── */
void loop() {
    // Read sensors every iteration
    updateOdometry();
    computeMeasurementPoint();
    updateWheelVelocities();

    // State machine
    switch (navState) {

    /* ── IDLE: wait for start button press ── */
    case NAV_IDLE: {
        if (digitalRead(PIN_BUTTON) == LOW) {
            delay(50);  // simple debounce
            if (digitalRead(PIN_BUTTON) == LOW) {
                startTimeMs = millis();
                navState = NAV_STRAIGHT;
                currentWP = 1;
                desiredHeading = headingTo(measX, measY, wpX[1], wpY[1]);
                headingPID.reset();
                velPID_L.reset();
                velPID_R.reset();
                Serial.println(F(">>> STARTED <<<"));
            }
        }
        break;
    }

    /* ── STRAIGHT: drive toward current waypoint using cascaded PID ── */
    case NAV_STRAIGHT: {
        double distToWP = dist2D(measX, measY, wpX[currentWP], wpY[currentWP]);

        // Check if we entered the final-approach zone of the LAST waypoint
        if (currentWP == NUM_WAYPOINTS - 1 && distToWP <= SLOW_DIST_MM) {
            navState = NAV_FINAL_APPROACH;
            Serial.println(F(">>> FINAL APPROACH <<<"));
            break;
        }

        // Check arrival at current waypoint
        if (distToWP < WP_ARRIVE_THRESH) {
            Serial.print(F("Arrived at WP")); Serial.println(currentWP);

            if (currentWP >= NUM_WAYPOINTS - 1) {
                navState = NAV_DONE;
                break;
            }

            // Begin turning toward next waypoint
            navState = NAV_TURNING;
            desiredHeading = headingTo(wpX[currentWP], wpY[currentWP],
                                       wpX[currentWP + 1], wpY[currentWP + 1]);
            headingPID.reset();
            break;
        }

        // ── Outer loop: heading PID ──
        // Continuously point toward the current target waypoint
        double targetHeading = headingTo(measX, measY, wpX[currentWP], wpY[currentWP]);
        double headingError  = wrapAngle(targetHeading - odomHeading);

        // Heading PID outputs a differential velocity correction (mm/s)
        double headingCorrection = headingPID.compute(0.0, -headingError);

        // Base forward speed for straight segments
        double baseSpeed = CRUISE_SPEED_MM_S;

        // Split into left/right wheel velocity targets
        double targetVelL = baseSpeed - headingCorrection;
        double targetVelR = baseSpeed + headingCorrection;

        // ── Inner loops: velocity PIDs (one per wheel) ──
        double cmdL = velPID_L.compute(targetVelL, wheelVelL);
        double cmdR = velPID_R.compute(targetVelR, wheelVelR);

        setServoOutput(cmdL, cmdR);
        break;
    }

    /* ── TURNING: in-place rotation at a waypoint ── */
    case NAV_TURNING: {
        double headingError = wrapAngle(desiredHeading - odomHeading);

        if (fabs(headingError) < TURN_THRESH) {
            // Turn complete — advance to next waypoint
            currentWP++;
            navState = NAV_STRAIGHT;
            headingPID.reset();
            velPID_L.reset();
            velPID_R.reset();
            Serial.print(F("Turn done, heading to WP")); Serial.println(currentWP);
            break;
        }

        // Differential drive: wheels spin in opposite directions to rotate
        double headingCorrection = headingPID.compute(0.0, -headingError);

        double targetVelL = -headingCorrection;
        double targetVelR =  headingCorrection;

        // Clamp turning speed for accuracy
        targetVelL = constrain(targetVelL, -TURN_SPEED_MM_S, TURN_SPEED_MM_S);
        targetVelR = constrain(targetVelR, -TURN_SPEED_MM_S, TURN_SPEED_MM_S);

        double cmdL = velPID_L.compute(targetVelL, wheelVelL);
        double cmdR = velPID_R.compute(targetVelR, wheelVelR);

        setServoOutput(cmdL, cmdR);
        break;
    }

    /* ── FINAL APPROACH: adaptive speed to hit finish at exactly TARGET_TIME ── */
    case NAV_FINAL_APPROACH: {
        double distToFinish = dist2D(measX, measY,
                                     wpX[NUM_WAYPOINTS - 1], wpY[NUM_WAYPOINTS - 1]);

        // Check arrival at finish
        if (distToFinish < WP_ARRIVE_THRESH) {
            navState = NAV_DONE;
            break;
        }

        // Compute remaining time
        double elapsedS   = (double)(millis() - startTimeMs) / 1000.0;
        double remainingS = TARGET_TIME_S - elapsedS;

        // Decide speed based on the strategy:
        //   (a) Enough time remaining -> slow down to arrive exactly on time
        //   (b) Not enough time       -> keep cruise speed
        double finalSpeed;

        if (remainingS > 0.0) {
            double timeAtCruise = distToFinish / CRUISE_SPEED_MM_S;

            if (timeAtCruise < remainingS) {
                // Case (a): extra time — slow down to use exactly remainingS
                finalSpeed = distToFinish / remainingS;
                if (finalSpeed < MIN_FINAL_SPEED_MM_S) finalSpeed = MIN_FINAL_SPEED_MM_S;
            } else {
                // Case (b): behind schedule — maintain cruise speed
                finalSpeed = CRUISE_SPEED_MM_S;
            }
        } else {
            // Past target time — keep going at cruise speed
            finalSpeed = CRUISE_SPEED_MM_S;
        }

        // Heading correction toward finish point
        double targetHeading = headingTo(measX, measY,
                                         wpX[NUM_WAYPOINTS - 1], wpY[NUM_WAYPOINTS - 1]);
        double headingError  = wrapAngle(targetHeading - odomHeading);
        double headingCorrection = headingPID.compute(0.0, -headingError);

        double targetVelL = finalSpeed - headingCorrection;
        double targetVelR = finalSpeed + headingCorrection;

        double cmdL = velPID_L.compute(targetVelL, wheelVelL);
        double cmdR = velPID_R.compute(targetVelR, wheelVelR);

        setServoOutput(cmdL, cmdR);
        break;
    }

    /* ── DONE: stop motors, report results, return to idle ── */
    case NAV_DONE: {
        stopMotors();

        double totalTimeS = (double)(millis() - startTimeMs) / 1000.0;
        Serial.println(F("=== FINISHED ==="));
        Serial.print(F("Total time: "));  Serial.print(totalTimeS);  Serial.println(F(" s"));
        Serial.print(F("Target time: ")); Serial.print(TARGET_TIME_S); Serial.println(F(" s"));
        Serial.print(F("Difference: "));  Serial.print(totalTimeS - TARGET_TIME_S); Serial.println(F(" s"));
        Serial.print(F("Final meas point: (")); Serial.print(measX);
        Serial.print(F(", ")); Serial.print(measY); Serial.println(F(")"));

        navState = NAV_IDLE;   // allow restart by pressing button again
        break;
    }

    } // end switch
}