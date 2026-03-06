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
 *      1. Positional PID (outer loop) — uses goBILDA Pinpoint Odometry Computer
 *         (via the goBILDA_Pinpoint library) to obtain the robot's (x, y, heading)
 *         pose and produces a desired forward velocity and a heading-correction term.
 *      2. Velocity PIDs (inner loops, one per wheel) — each reads a
 *         600 PP/R quadrature encoder (via Paul Stoffregen's Encoder library),
 *         computes the current wheel speed, and drives a goBILDA continuous-
 *         rotation servo to match the velocity set-point produced by the outer loop.
 *
 *  All three PID controllers use Brett Beauregard's PID_v1 library.
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
 *      I2C (A4/A5) – goBILDA Pinpoint Odometry Computer
 *
 *  LIBRARIES
 *  ---------
 *      • Servo.h              – built-in Arduino servo control
 *      • Encoder.h            – Paul Stoffregen's quadrature encoder library
 *      • PID_v1.h             – Brett Beauregard's PID library (v1.2.1)
 *      • goBILDA_Pinpoint.h   – goBILDA official Pinpoint Odometry Computer library
 *
 * =============================================================================
 */

#include <Servo.h>
#include <Wire.h>
#include <Encoder.h>             // Paul Stoffregen's Encoder library (quadrature decoding)
#include <PID_v2.h>              // Brett Beauregard's PID library (v1.2.1)
#include <goBILDA_Pinpoint.h>    // goBILDA official Pinpoint Odometry Computer library

/* ─────────────────────────────────────────────────────────────────────────────
 *  goBILDA Pinpoint Odometry Computer
 *  Provides fused (x, y, heading) pose via I2C using two dead-wheel pods
 *  and an internal IMU.  The library handles all I2C communication internally.
 * ───────────────────────────────────────────────────────────────────────────── */
goBILDA::Pinpoint pinpoint;

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
 *  ENCODER OBJECTS  (Paul Stoffregen's Encoder library)
 *
 *  The library handles quadrature decoding and interrupt attachment internally.
 *  Best performance when channel-A pins support hardware interrupts
 *  (pins 2 & 3 on Arduino Uno); channel-B pins use change detection.
 * ───────────────────────────────────────────────────────────────────────────── */
Encoder encL(PIN_ENC_L_A, PIN_ENC_L_B);   // Left  wheel encoder
Encoder encR(PIN_ENC_R_A, PIN_ENC_R_B);   // Right wheel encoder

/* ─────────────────────────────────────────────────────────────────────────────
 *  ODOMETRY  (goBILDA Pinpoint library)
 *
 *  The Pinpoint Odometry Computer fuses two dead-wheel encoder pods with an
 *  internal IMU and reports (x, y, heading) over I2C.  The goBILDA_Pinpoint
 *  library handles all register reads and data conversion internally.
 * ───────────────────────────────────────────────────────────────────────────── */

// Robot pose from odometry computer (all in mm / radians)
double odomX       = 0.0;   // mm, field frame
double odomY       = 0.0;   // mm, field frame
double odomHeading = 0.0;   // radians

// Read latest pose from the goBILDA Pinpoint Odometry Computer
void updateOdometry() {
    goBILDA::Pose2D pos = pinpoint.getPosition();
    odomX       = (double)pos.x;          // mm
    odomY       = (double)pos.y;          // mm
    odomHeading = (double)pos.heading;    // radians
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
 *  PID CONTROLLERS  (Brett Beauregard's PID_v1 library)
 *
 *  The PID_v1 library links each controller to three double variables
 *  (Input, Output, Setpoint) via pointers.  Call Compute() every loop
 *  iteration; the library internally enforces the sample time.
 *
 *  Outer loop:  headingPID — error = desired heading − current heading
 *  Inner loops: velPID_L, velPID_R — one per wheel
 *
 *  Tuning parameters are starting points; adjust on the real robot.
 * ───────────────────────────────────────────────────────────────────────────── */

// Heading PID tuning: input = rad, output = mm/s differential correction
const double HDG_KP = 150.0;
const double HDG_KI =   5.0;
const double HDG_KD =  10.0;

// Velocity PID tuning: input = mm/s error, output = servo microseconds offset
// Split per wheel so each side can be tuned independently.
const double VEL_KP_L = 0.8;
const double VEL_KI_L = 2.0;
const double VEL_KD_L = 0.01;

const double VEL_KP_R = 0.8;
const double VEL_KI_R = 2.0;
const double VEL_KD_R = 0.01;

// PID I/O variables — the PID_v1 library reads/writes these via pointers
double hdgInput = 0.0, hdgOutput = 0.0, hdgSetpoint = 0.0;
double velInputL = 0.0, velOutputL = 0.0, velSetpointL = 0.0;
double velInputR = 0.0, velOutputR = 0.0, velSetpointR = 0.0;

// Construct PID objects (DIRECT = positive output for positive error)
PID headingPID(&hdgInput, &hdgOutput, &hdgSetpoint, HDG_KP, HDG_KI, HDG_KD, DIRECT);
PID velPID_L(&velInputL, &velOutputL, &velSetpointL, VEL_KP_L, VEL_KI_L, VEL_KD_L, DIRECT);
PID velPID_R(&velInputR, &velOutputR, &velSetpointR, VEL_KP_R, VEL_KI_R, VEL_KD_R, DIRECT);

/*
 * resetPID() — helper to reset a PID controller's internal state.
 * Toggles mode MANUAL→AUTOMATIC which clears the integral accumulator
 * and resets the derivative history inside the PID_v1 library.
 */
void resetPID(PID &pid, double &output) {
    pid.SetMode(MANUAL);
    output = 0.0;
    pid.SetMode(AUTOMATIC);
}

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

// Compute wheel velocities from encoder counts (using Encoder library)
void updateWheelVelocities() {
    unsigned long now = micros();
    double dt = (double)(now - velLastTime) / 1e6;
    if (dt < 0.005) return;  // don't update faster than every 5 ms
    velLastTime = now;

    // Read encoder counts via the Encoder library (thread-safe internally)
    long cL = encL.read();
    long cR = encR.read();

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

    // Initialise goBILDA Pinpoint Odometry Computer (I2C is started internally)
    pinpoint.begin();
    pinpoint.setOffsets(0.0, 125.7);
    pinpoint.setEncoderResolution(goBILDA::EncoderResolution::goBILDA_4_BAR_POD);
    pinpoint.setEncoderDirections(goBILDA::EncoderDirection::Forward,
                                  goBILDA::EncoderDirection::Forward);
    pinpoint.resetPositionAndIMU();

    // Encoder library handles pin modes and interrupts internally — no manual
    // pinMode() or attachInterrupt() calls are needed for encoder pins.

    // Start button
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    // Servos
    servoL.attach(PIN_SERVO_L);
    servoR.attach(PIN_SERVO_R);
    stopMotors();

    // Configure PID controllers (PID_v1 library)
    // Sample time set to 10 ms: fast enough for responsive heading and velocity
    // corrections while remaining well above the 5 ms minimum velocity-measurement
    // update interval in updateWheelVelocities().
    headingPID.SetMode(AUTOMATIC);
    headingPID.SetOutputLimits(-200.0, 200.0);
    headingPID.SetSampleTime(10);

    velPID_L.SetMode(AUTOMATIC);
    velPID_L.SetOutputLimits(-500.0, 500.0);
    velPID_L.SetSampleTime(10);

    velPID_R.SetMode(AUTOMATIC);
    velPID_R.SetOutputLimits(-500.0, 500.0);
    velPID_R.SetSampleTime(10);

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
                resetPID(headingPID, hdgOutput);
                resetPID(velPID_L, velOutputL);
                resetPID(velPID_R, velOutputR);
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
            resetPID(headingPID, hdgOutput);
            break;
        }

        // ── Outer loop: heading PID ──
        // Continuously point toward the current target waypoint
        double targetHeading = headingTo(measX, measY, wpX[currentWP], wpY[currentWP]);
        double headingError  = wrapAngle(targetHeading - odomHeading);

        // Heading PID: setpoint = 0, input = negative heading error
        hdgSetpoint = 0.0;
        hdgInput    = -headingError;
        headingPID.Compute();
        double headingCorrection = hdgOutput;

        // Base forward speed for straight segments
        double baseSpeed = CRUISE_SPEED_MM_S;

        // Split into left/right wheel velocity targets
        double targetVelL = baseSpeed - headingCorrection;
        double targetVelR = baseSpeed + headingCorrection;

        // ── Inner loops: velocity PIDs (one per wheel) ──
        velSetpointL = targetVelL;
        velInputL    = wheelVelL;
        velPID_L.Compute();

        velSetpointR = targetVelR;
        velInputR    = wheelVelR;
        velPID_R.Compute();

        setServoOutput(velOutputL, velOutputR);
        break;
    }

    /* ── TURNING: in-place rotation at a waypoint ── */
    case NAV_TURNING: {
        double headingError = wrapAngle(desiredHeading - odomHeading);

        if (fabs(headingError) < TURN_THRESH) {
            // Turn complete — advance to next waypoint
            currentWP++;
            navState = NAV_STRAIGHT;
            resetPID(headingPID, hdgOutput);
            resetPID(velPID_L, velOutputL);
            resetPID(velPID_R, velOutputR);
            Serial.print(F("Turn done, heading to WP")); Serial.println(currentWP);
            break;
        }

        // Differential drive: wheels spin in opposite directions to rotate
        hdgSetpoint = 0.0;
        hdgInput    = -headingError;
        headingPID.Compute();
        double headingCorrection = hdgOutput;

        double targetVelL = -headingCorrection;
        double targetVelR =  headingCorrection;

        // Clamp turning speed for accuracy
        targetVelL = constrain(targetVelL, -TURN_SPEED_MM_S, TURN_SPEED_MM_S);
        targetVelR = constrain(targetVelR, -TURN_SPEED_MM_S, TURN_SPEED_MM_S);

        velSetpointL = targetVelL;
        velInputL    = wheelVelL;
        velPID_L.Compute();

        velSetpointR = targetVelR;
        velInputR    = wheelVelR;
        velPID_R.Compute();

        setServoOutput(velOutputL, velOutputR);
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

        hdgSetpoint = 0.0;
        hdgInput    = -headingError;
        headingPID.Compute();
        double headingCorrection = hdgOutput;

        double targetVelL = finalSpeed - headingCorrection;
        double targetVelR = finalSpeed + headingCorrection;

        velSetpointL = targetVelL;
        velInputL    = wheelVelL;
        velPID_L.Compute();

        velSetpointR = targetVelR;
        velInputR    = wheelVelR;
        velPID_R.Compute();

        setServoOutput(velOutputL, velOutputR);
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