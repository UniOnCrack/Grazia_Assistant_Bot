#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS5Controller.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 150 // Minimum pulse length count (adjust as needed)
#define SERVOMAX 600 // Maximum pulse length count (adjust as needed)

// Servo channels on PCA9685
#define SERVO_1_CHANNEL 0
#define SERVO_2_CHANNEL 8
#define SERVO_3_CHANNEL 15

// Servo angles
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

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz

  PS5.attach(notify);
  PS5.begin("10:18:49:7d:6d:f4");

  resetServos();
}

void loop() {
  delay(10);
}

void notify() {
  int rStickY = PS5.RStickY();
  
  if (PS5.R3()) {
    if (rStickY > 100) {  // R-stick up
      moveForward();
    } else if (rStickY < -100) {  // R-stick down
      moveBackward();
    }
  } else if (PS5.Circle()) {
    resetServos();
  }
}

void moveForward() {
  // Move servo 1 clockwise within range of 45° to 90°
  servo1Angle = constrain(servo1Angle + 45, servo1Min, servo1Max);
  pwm.setPWM(SERVO_1_CHANNEL, 0, angleToPulse(servo1Angle));

  // Move servo 2 counter-clockwise within range of 90° to 0°
  servo2Angle = constrain(servo2Angle - 90, servo2Min, servo2Max);
  pwm.setPWM(SERVO_2_CHANNEL, 0, angleToPulse(servo2Angle));

  // Move servo 3 clockwise within range of -45° to 0°
  servo3Angle = constrain(servo3Angle + 45, servo3Min, servo3Max);
  pwm.setPWM(SERVO_3_CHANNEL, 0, angleToPulse(servo3Angle));
}

void moveBackward() {
  // Move servo 1 counter-clockwise within range of 45° to 0°
  servo1Angle = constrain(servo1Angle - 45, servo1Min, servo1Max);
  pwm.setPWM(SERVO_1_CHANNEL, 0, angleToPulse(servo1Angle));

  // Move servo 2 clockwise within range of 90° to 185°
  servo2Angle = constrain(servo2Angle + 95, servo2Min, servo2Max);
  pwm.setPWM(SERVO_2_CHANNEL, 0, angleToPulse(servo2Angle));

  // Move servo 3 counter-clockwise within range of -45° to -90°
  servo3Angle = constrain(servo3Angle - 45, servo3Min, servo3Max);
  pwm.setPWM(SERVO_3_CHANNEL, 0, angleToPulse(servo3Angle));
}

void resetServos() {
  // Reset all servos to their initial positions
  servo1Angle = 45;
  servo2Angle = 90;
  servo3Angle = -45;
  pwm.setPWM(SERVO_1_CHANNEL, 0, angleToPulse(servo1Angle));
  pwm.setPWM(SERVO_2_CHANNEL, 0, angleToPulse(servo2Angle));
  pwm.setPWM(SERVO_3_CHANNEL, 0, angleToPulse(servo3Angle));
}

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}