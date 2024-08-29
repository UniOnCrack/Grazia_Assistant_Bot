#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create the PWM driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150  // Minimum pulse length count (corresponds to 0 degrees)
#define SERVOMAX  600  // Maximum pulse length count (corresponds to 180 degrees)

// Servo channel on the PCA9685
#define SERVO1_CHANNEL 0

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing PCA9685...");

  // Initialize the PWM driver
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz

  // Wait for the driver to initialize
  delay(10);

  Serial.println("Enter servo angle (0, 90, 180):");
}

void loop() {
  if (Serial.available()) {
    int angle = Serial.parseInt();  // Read the angle from serial input

    if (angle == 0 || angle == 90 || angle == 180) {
      Serial.print("Moving Servo to ");
      Serial.print(angle);
      Serial.println(" degrees");

      moveServoToAngle(angle);  // Move the servo to the specified angle
    } else {
      Serial.println("Invalid input. Please enter 0, 90, or 180.");
    }
  }
}

void moveServoToAngle(int angle) {
  uint16_t pulseLength = SERVOMIN + ((SERVOMAX - SERVOMIN) * angle / 180);
  pwm.setPWM(SERVO1_CHANNEL, 0, pulseLength);  // Move Servo to the specified angle
}
