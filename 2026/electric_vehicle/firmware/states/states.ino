#include <Encoder.h>
#include <PID_v2.h>
#include <goBILDA_Pinpoint.h>

// Pins
#define PIN_MOTOR_1 9 // Super Speed
#define PIN_MOTOR_2 10 // Speed
#define PIN_ENCODER_1 2
#define PIN_ENCODER_2 3

// Hardware specifications
const double WHEEL_CIRCUMFERENCE_CM = 12.0; 
const int TICKS_PER_REVOLUTION = 600; // How many ticks your encoder outputs per full spin
const double CM_PER_TICK = WHEEL_CIRCUMFERENCE_CM / TICKS_PER_REVOLUTION;

// Velocity PID
double Kp_V1 = 2.0, Ki_V1 = 5.0, Kd_V1 = 1.0; // These will need to be tuned!
double Kp_V2 = 2.0, Ki_V2 = 5.0, Kd_V2 = 1.0; // These will need to be tuned!
double targetVelocityCMS = 30.0; // SETPOINT: Desired speed in cm/s

volatile long encoderTicks = 0;   // Volatile because it changes inside an interrupt
unsigned long previousMillis = 0;
const int calcIntervalMs = 100;   // Calculate speed every 100 milliseconds

// Position PID
double Kp_P = 2.0, Ki_P = 5.0, Kd_P = 1.0; // These will need to be tuned!

// Class setup
goBILDA::Pinpoint pinpoint;
goBILDA::Pose2D position;

Encoder Encoder1(2, 4); 
Encoder Encoder2(3, 5);
long oldPosition1  = -999;
long oldPosition2  = -999;

PID_v2 myPID_V1(Kp_V1, Ki_V1, Kd_V1, PID::Direct);
PID_v2 myPID_V2(Kp_V2, Ki_V2, Kd_V2, PID::Direct);
PID_v2 myPID_P(Kp_P, Ki_P, Kd_P, PID::Direct);


// Interrupt Service Routine (ISR) to count encoder ticks
void countTicks() {
  encoderTicks++;
}

void setup(){
  // Pins
  pinMode(PIN_MOTOR_1, OUTPUT);
  pinMode(PIN_MOTOR_2, OUTPUT);
  pinMode(PIN_ENCODER_1, INPUT_PULLUP);
  pinMode(PIN_ENCODER_2, INPUT_PULLUP);

  // Gobilda Odometry Setup
	pinpoint.begin();
  pinpoint.setOffsets(0.0, 185.7);                                                                        // X/Y offset from the robots center (millimeters)
  pinpoint.setEncoderResolution(goBILDA::EncoderResolution::goBILDA_4_BAR_POD);                           // Set the encoder resolution        (millimeters per tick)
  pinpoint.setEncoderDirections(goBILDA::EncoderDirection::Forward, goBILDA::EncoderDirection::Forward); //SET LATER // Set X and Y encoder directions

  // PID Setup
  myPID_V1.Start(0.0, 0.0, targetVelocityCMS);
  myPID_V2.Start(0.0, 0.0, targetVelocityCMS);
  myPID_P.Start(0.0, 0.0, targetVelocityCMS);

  // Encoder Setup
  

  // Button Setup

}

void loop() {
  unsigned long currentMillis = millis();
  
  // Gobilda Odometry Loop
  position = pinpoint.getPosition();

  // PID Loop
  if (currentMillis - previousMillis >= calcIntervalMs) {
    
    // 1. Calculate actual velocity in cm/s
    //    Distance = ticks * cm_per_tick
    //    Time (in seconds) = calcIntervalMs / 1000.0
    double distanceTraveled = encoderTicks * CM_PER_TICK;
    double actualVelocityCMS = distanceTraveled / (calcIntervalMs / 1000.0);
    
    // Reset ticks and timer for the next loop
    encoderTicks = 0; 
    previousMillis = currentMillis;

    // 2. Compute the PID output (PWM)
    //    Pass the actual velocity as the input. The PID returns the required PWM.
    double motorPWM = myPID.Run(actualVelocityCMS);

    // Constrain the output just to be safe (PWM must be 0-255)
    motorPWM = constrain(motorPWM, 0, 255);

    // 3. Apply the PWM signal to the motor driver
    analogWrite(PIN_MOTOR_PWM, motorPWM);
  }

  // Encoder Loop
  // Read the current positions
  long newPosition1 = Encoder1.read();
  long newPosition2 = Encoder2.read();

  // If the position of either encoder has changed, print it out
  if (newPosition1 != oldPosition1 || newPosition2 != oldPosition2) {
    oldPosition1 = newPosition1;
    oldPosition2 = newPosition2;
  }

  // Button Loop

}
