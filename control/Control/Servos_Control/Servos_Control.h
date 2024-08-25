// ServoControl.h
#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create the PWM driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150  // Minimum pulse length count (corresponds to 0 degrees)
#define SERVOMAX  600  // Maximum pulse length count (corresponds to 180 degrees)

// Servo channels on the PCA9685
#define SERVO1_CHANNEL 0
#define SERVO2_CHANNEL 15

void initServos() {
  Serial.println("Initializing PCA9685...");

  // Initialize the PWM driver
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz

  // Wait for the driver to initialize
  delay(10);
}

void moveServosTo90Degrees() {
  uint16_t pulse90 = SERVOMIN + ((SERVOMAX - SERVOMIN) * 90 / 180);

  pwm.setPWM(SERVO1_CHANNEL, 0, pulse90);  // Move Servo 1 to 90 degrees
  pwm.setPWM(SERVO2_CHANNEL, 0, pulse90);  // Move Servo 2 to 90 degrees
}

void moveServosTo0Degrees() {
  pwm.setPWM(SERVO1_CHANNEL, 0, SERVOMIN);  // Move Servo 1 to 0 degrees
  pwm.setPWM(SERVO2_CHANNEL, 0, SERVOMIN);  // Move Servo 2 to 0 degrees
}

#endif
