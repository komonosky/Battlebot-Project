// ME 464 Combat Robot Project
// Author: Sam Komonosky
// Date: May 5, 2023

#include <AFMotor.h>    // Adafruit Motor Shield library V1 - used with L293D motor driver shield
#include <TFLI2C.h>     // TF-Luna I2C library - used for the LiDAR sensor
#include <Wire.h>       // allows communication with I2C devices with Arduino MEGA

TFLI2C tfli2c;                  // initialize TF-Luna I2C object
AF_DCMotor weaponMotor(4);      // Initialize weapon motor - M4 on L293D driver
AF_DCMotor leftMotor(2);        // Initialize left DC motor - M2 on L293D driver
AF_DCMotor rightMotor(3);       // Initialize right DC motor - M3 on L293D driver
int irLeft = 47;                // Left infrared sensor - Digital Output Pin 47
int irRight = 49;               // Right infrared sensor - Digital Output Pin 49
int qti1 = 39;                  // QTI sensor - Digital Output Pin 39

int16_t tfAddr = TFL_DEF_ADR;     // Default I2C address
int16_t tfDist;                   // Distance calculated by LiDAR (cm)

int LEDPin = 44;                // Red LED - Digital Output Pin 44

// Function declarations
int speedPercentage(int);
void moveForward();
void rotateRight();
void rotateLeft();
void reverse();
void stop();
void weapon();

void setup() {
  Serial.begin(115200);         // Initialize serial port
  Wire.begin();                 // Initialize Wire library
  pinMode(LEDPin, OUTPUT);
  digitalWrite(LEDPin, LOW);    // Turns LED off initially
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
  pinMode(qti1, OUTPUT);        // charge QTI capacitor
  digitalWrite(qti1, HIGH);
  delay(1);
  pinMode(qti1, INPUT);         // discharge QTI capacitor
  delay(1);
  delay(10000);                 // 10 second delay to get the robot in the ring
}

void loop() {
  // If data is read without error:
  if (tfli2c.getData(tfDist, tfAddr)) {
    byte qtiSensor1 = digitalRead(qti1);  // 0 = white, 1 = black
    int irStatusLeft = digitalRead(irLeft); // 0 = white, 1 = black
    int irStatusRight = digitalRead(irRight); // 0 = white, 1 = black

    if (qtiSensor1 == 1) {          // If QTI sensor senses black line, robot is out of bounds
      digitalWrite(LEDPin, HIGH);   // Turn on the red LED
      stop();                       // Stop the robot
      delay(30000);                 // delay for 30 seconds to safely remove robot from the ring
    } else if (irStatusLeft == 0 && irStatusRight == 1) {   // if Right IR sensor senses black line:
      rotateLeft();                                         // Turn left to remain in bounds
    } else if (irStatusLeft == 1 && irStatusRight == 0) {   // if Left IR sensor senses black line:
      rotateRight();                                        // Turn right to remain in bounds
    } else if (irStatusLeft == 1 && irStatusRight == 1) {   // If both IR sensors sense the line:
      reverse();                                            // Reverse
    } else if (tfDist > 100) {    // If distance detected by LiDAR is greater than 100 cm (no opponent detected):
      rotateRight();              // rotate the robot until it detects something
    } else if (tfDist < 50) {     // if distance detected is less than 50 cm:
      weapon();                   // activate the weapon and move forward
      moveForward();
    } else {                      // if opponent detected (distance between 50 and 100 cm):
      weaponMotor.run(RELEASE);   // stop the weapon motor if it was running
      moveForward();              // move toward the opponent
    }
  }
  delay(50);
}

// FUNCTION DEFINITIONS
// Function Name: speedPercentage
// Purpose: the Adafruit Motor Shield library's setSpeed function uses values
//          between 0 and 255 for motor speed, with 255 being maximum speed.
//          This maps those values to a percentage scale of 0 to 100, which is
//          easier to understand.
int speedPercentage(int percent) {
  return map(percent, 0, 100, 0, 255);
}

// Function Name: moveForward
// Purpose: Moves the robot forward. Sets both left and right DC motors 
//          to full speed and sets them in the forward direction.
void moveForward() {
  leftMotor.setSpeed(speedPercentage(100));
  rightMotor.setSpeed(speedPercentage(100));
  leftMotor.run(FORWARD);
  rightMotor.run(FORWARD);
}

// Function Name: rotateLeft
// Purpose: Turns the robot to the left. Achieves a zero-turn radius by
//          having the right motor move in the forward direction and the
//          left motor move backward. Speed is set to 80% so the LiDAR is able
//          to sense the opponent an stop turning (at full speed it just kept
//          spinning around in a cricle).
void rotateLeft() {
  leftMotor.setSpeed(speedPercentage(80));
  rightMotor.setSpeed(speedPercentage(80));
  leftMotor.run(BACKWARD);
  rightMotor.run(FORWARD);
}

// Function Name: rotateRight
// Purpose: Turns the robot to the right. Achieves a zero-turn radius by
//          having the left motor move in the forward direction and the
//          right motor move backward. Speed is set to 80% so the LiDAR is able
//          to sense the opponent an stop turning (at full speed it just kept
//          spinning around in a cricle).
void rotateRight() {
  leftMotor.setSpeed(speedPercentage(80));
  rightMotor.setSpeed(speedPercentage(80));
  leftMotor.run(FORWARD);
  rightMotor.run(BACKWARD);
}

// Function Name: reverse
// Purpose: Moves the robot backwards. Sets both left and right motors to full speed
//          and sets the direction to backward.
void reverse() {
  leftMotor.setSpeed(speedPercentage(100));
  rightMotor.setSpeed(speedPercentage(100));
  leftMotor.run(BACKWARD);
  rightMotor.run(BACKWARD);
}

// Function Name: stop
// Purpose: Stops the robot from moving by releasing both left and right DC motors.
void stop() {
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);
}

// Function Name: weapon
// Purpose: Activates the weapon. Sets the weapon motor to full speed
//          and runs it forward, then backward for 0.5 seconds each.
//          This is mostly to meet competition requirements. The weapon itself
//          is not going have much effect.
void weapon() {
  weaponMotor.setSpeed(speedPercentage(100));
  weaponMotor.run(FORWARD);
  delay(500);
  weaponMotor.run(BACKWARD);
  delay(500);
}
