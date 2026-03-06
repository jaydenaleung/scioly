/*
  goBILDA Pinpoint - Basic Position Example
  -----------------------------------------
  This example demonstrates how to initialize the goBILDA Pinpoint Odometry Computer
  and read the fused position and heading data using the goBILDA_Pinpoint Arduino library.

  The Pinpoint Odometry Computer combines encoder data from two tracking pods and IMU
  sensor fusion to calculate real-time X, Y, and heading (H) values for your robot.

  Hardware connections:
    - Pinpoint I2C -> Your 3.3V microcontrollers I2C
    - Pinpoint X   -> X-Encoder 
    - Pinpoint Y   -> Y-Encoder

  This sketch prints the position data to the Serial Monitor every 100 milliseconds
  in the following format:

    "X: 0.123, Y: 1.234, H: 2.345"

  Where:
    X = X-axis position in millimeters
    Y = Y-axis position in millimeters
    H = Heading angle in degrees
*/

#include <goBILDA_Pinpoint.h>

goBILDA::Pinpoint pinpoint;

void setup() {
  Serial.begin(115200);
  pinpoint.begin();
  pinpoint.setEncoderDirections(goBILDA::EncoderDirection::Forward, goBILDA::EncoderDirection::Forward);  // Set X and Y encoder directions
  pinpoint.setEncoderResolution(goBILDA::EncoderResolution::goBILDA_4_BAR_POD);                           // Set the encoder resolution
}

void loop() {
  // Wait 100 milliseconds between updates
  delay(100);

  // Retrieves the latest position and heading from the Pinpoint
  // Pose2D is a struct containing x, y, and heading values
  goBILDA::Pose2D position = pinpoint.getPosition();

  // Display the data to the serial monitor in a readable format
  Serial.print("X: ");
  Serial.print(position.x, 3);  // only show 3 decimal places
  
  Serial.print(", Y: ");
  Serial.print(position.y, 3);

  Serial.print(", H: ");
  Serial.print(position.heading, 3);

  /*
    Example output (every 100ms):

    "X: 0.123, Y: 1.234, H: 2.345"

    This shows the Pinpoint's calculated position and heading.
  */
}