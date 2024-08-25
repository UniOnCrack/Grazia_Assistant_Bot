// MotorControl.h
#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#define IN1 14   // GPIO pin for IN1
#define IN2 12   // GPIO pin for IN2
#define IN3 26   // GPIO pin for IN3
#define IN4 25   // GPIO pin for IN4

void initMotors() {
  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void moveMotorsForward() {
  // Motor A forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Motor B forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveMotorsBackward() {
  // Motor A backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Motor B backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

#endif
