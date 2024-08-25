#include <ps5Controller.h>
#include "Motors_Control.h"

unsigned long lastTimeStamp = 0;

// Function to handle PS5 controller input and control motors
void notify() {
  int lx = ps5.LStickX();  // Left stick X-axis
  int ly = ps5.LStickY();  // Left stick Y-axis

  // Debugging: Print joystick values
  Serial.print("LX: ");
  Serial.print(lx);
  Serial.print(" LY: ");
  Serial.println(ly);

  // Control logic for motors based on stick input
  // Stick up: Both motors move forward
  if (ly > 100 && abs(lx) < 100) {
    moveMotorsForward();
  }
  // Stick down: Both motors move backward
  else if (ly < -100 && abs(lx) < 100) {
    moveMotorsBackward();
  }
  // Stick left: Motor 1 moves forward, Motor 2 moves backward
  else if (lx < -100 && abs(ly) < 100) {
    motor1Forward();
    motor2Backward();
  }
  // Stick right: Motor 2 moves forward, Motor 1 moves backward
  else if (lx > 100 && abs(ly) < 100) {
    motor2Forward();
    motor1Backward();
  }
  // Stick up-left: Motor 1 moves forward, Motor 2 moves forward at 50% speed
  else if (ly > 100 && lx < -100) {
    motor1Forward();
    analogWrite(IN3, 128);  // Set Motor 2 to 50% speed
    digitalWrite(IN4, LOW); // Ensure Motor 2 moves forward
  }
  // Stick up-right: Motor 2 moves forward, Motor 1 moves forward at 50% speed
  else if (ly > 100 && lx > 100) {
    motor2Forward();
    analogWrite(IN1, 128);  // Set Motor 1 to 50% speed
    digitalWrite(IN2, LOW); // Ensure Motor 1 moves forward
  }
  // Stick down-left: Motor 1 moves backward, Motor 2 moves backward at 50% speed
  else if (ly < -100 && lx < -100) {
    motor1Backward();
    analogWrite(IN3, 128);  // Set Motor 2 to 50% speed
    digitalWrite(IN4, HIGH); // Ensure Motor 2 moves backward
  }
  // Stick down-right: Motor 2 moves backward, Motor 1 moves backward at 50% speed
  else if (ly < -100 && lx > 100) {
    motor2Backward();
    analogWrite(IN1, 128);  // Set Motor 1 to 50% speed
    digitalWrite(IN2, HIGH); // Ensure Motor 1 moves backward
  }
  // Stop motors if no valid input is detected
  else {
    stopMotors();
  }
}

void setup() {
  Serial.begin(115200);
  ps5.attach(notify);
  ps5.begin("xx:xx:xx:xx:xx:xx");  // Replace with your controller's MAC address

  initMotors();  // Initialize motors
}

void loop() {
  // Main loop to handle PS5 input
  delay(10);
}
