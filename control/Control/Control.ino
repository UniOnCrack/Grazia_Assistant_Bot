#include <ps5Controller.h>
#include "Motor_Control\Motors_Control.h"

unsigned long lastTimeStamp = 0;

// Function to handle PS5 controller input and control motors
void notify() {
  int lx = ps5.LStickX();
  int ly = ps5.LStickY();

  // Print joystick values
  Serial.print("LX: ");
  Serial.print(lx);
  Serial.print(" LY: ");
  Serial.println(ly);

  // Motor 1 = Right Motor
  // Motor 2 = Left Motor

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
  // Stop motors if no input
  else {
    stopMotors();
  }
}

  // Setuo up for connect to controller
void setup() {
  Serial.begin(115200);
  ps5.attach(notify);
  ps5.begin("10:18:49:7d:6d:f4");

  initMotors();
}

void loop() {
  delay(10);
}
