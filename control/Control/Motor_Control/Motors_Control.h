#ifndef MOTORS_CONTROL_H
#define MOTORS_CONTROL_H

// Define motor control pins
#define IN1 14  // Motor 1 Forward
#define IN2 12  // Motor 1 Backward
#define IN3 26  // Motor 2 Forward
#define IN4 25  // Motor 2 Backward

// Stop all motors
void stopMotors() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
}

// Initialize motors 
void initMotors() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors(); // Ensure motors are stopped on initialization
}

// Move forward
void moveMotorsForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Move backward
void moveMotorsBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Motor 1 forward
void motor1Forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

// Motor 1 backward
void motor1Backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

// Motor 2 forward
void motor2Forward() {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Motor 2 backward
void motor2Backward() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

#endif
