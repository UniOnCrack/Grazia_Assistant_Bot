#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS5Controller.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 150 // Minimum pulse length count (adjust as needed)
#define SERVOMAX 600 // Maximum pulse length count (adjust as needed)

// Servo channels on PCA9685
#define SERVO_1_CHANNEL 0
#define SERVO_2_CHANNEL 1
#define SERVO_3_CHANNEL 2

// Initial Servo angles
int servo1Angle = 45;
int servo2Angle = 90;
int servo3Angle = -45;

// Constants for servo limits
int servo1Min = 0;
int servo1Max = 90;
int servo2Min = 0;
int servo2Max = 185;
int servo3Min = -90;
int servo3Max = 0;

// Convert angle to pulse length for PCA9685
int angleToPulse(int angle) {
  return map(angle, -90, 180, SERVOMIN, SERVOMAX);
}

// Function to move servos to their angles
void moveServos() {
  pwm.setPWM(SERVO_1_CHANNEL, 0, angleToPulse(servo1Angle));
  pwm.setPWM(SERVO_2_CHANNEL, 0, angleToPulse(servo2Angle));
  pwm.setPWM(SERVO_3_CHANNEL, 0, angleToPulse(servo3Angle));
}

// Reset servos to initial positions
void resetServos() {
  servo1Angle = 45;
  servo2Angle = 90;
  servo3Angle = -45;
  moveServos();
}

// Function to handle PS5 controller input and control servos
void notify() {
  if (ps5.RightStickY() < -100) { // Right stick down: Move servos back
    servo1Angle = max(servo1Min, servo1Angle - 45); // Move Servo 1 CCW (45° → 0°)
    servo2Angle = min(servo2Max, servo2Angle + 95); // Move Servo 2 CW (90° → 185°)
    servo3Angle = max(servo3Min, servo3Angle - 45); // Move Servo 3 CCW (-45° → -90°)
  } 
  else if (ps5.RightStickY() > 100) { // Right stick up: Move servos forward
    servo1Angle = min(servo1Max, servo1Angle + 45); // Move Servo 1 CW (45° → 90°)
    servo2Angle = max(servo2Min, servo2Angle - 90); // Move Servo 2 CCW (90° → 0°)
    servo3Angle = min(servo3Max, servo3Angle + 45); // Move Servo 3 CW (-45° → 0°)
  }

  // Update servos with new angles
  moveServos();

  // Reset servos to original state on pressing Triangle button
  if (ps5.Triangle()) {
    resetServos();
  }
}

// Setup for connection to the controller and PCA9685
void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60); // Set frequency to 60 Hz for servos

  // Connect to PS5 controller
  while (!ps5.isConnected()) {
    Serial.println("Attempting to connect to PS5 controller...");
    ps5.begin("10:18:49:7d:6d:f4");  // Replace with your controller's Bluetooth address
    delay(5000);  // Wait before trying to reconnect
  }

  // Attach the notify function once connected
  ps5.attach(notify);
  Serial.println("PS5 controller connected!");

  // Set servos to initial positions
  resetServos();
}

void loop() {
  // Handle PS5 controller events
  ps5.loop();
  delay(10);
}