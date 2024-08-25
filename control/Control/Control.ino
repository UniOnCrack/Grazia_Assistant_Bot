#include "Servos_Control\Servos_Control.h"
#include "Motor_Control\Motors_Control.h"
#include <ps5Controller.h>

unsigned long lastTimeStamp = 0;

void notify() {
  char messageString[200];
  sprintf(messageString, "%4d,%4d,%4d,%4d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d",
  ps5.LStickX(),
  ps5.LStickY(),
  ps5.RStickX(),
  ps5.RStickY(),
  ps5.Left(),
  ps5.Down(),
  ps5.Right(),
  ps5.Up(),
  ps5.Square(),
  ps5.Cross(),
  ps5.Circle(),
  ps5.Triangle(),
  ps5.L1(),
  ps5.R1(),
  ps5.L2(),
  ps5.R2(),  
  ps5.Share(),
  ps5.Options(),
  ps5.PSButton(),
  ps5.Touchpad(),
  ps5.Charging(),
  ps5.Audio(),
  ps5.Mic(),
  ps5.Battery());

  // Only needed to print the message properly on serial monitor. Else we don't need it.
  if (millis() - lastTimeStamp > 50) {
    Serial.println(messageString);
    lastTimeStamp = millis();
  }
}

void onConnect() {
  Serial.println("Connected!.");
}

void onDisConnect() {
  Serial.println("Disconnected!.");    
}

void setup() {
  Serial.begin(115200);
  
  // Initialize servos and motors
  initServos();
  initMotors();

  // Setup PS5 controller
  ps5.attach(notify);
  ps5.attachOnConnect(onConnect);
  ps5.attachOnDisconnect(onDisConnect);
  ps5.begin("10:18:49:7d:6d:f4");  // Replace with your PS5 controller MAC address
  
  while (ps5.isConnected() == false) { 
    Serial.println("PS5 controller not found");
    delay(300);
  }
  Serial.println("Ready.");
}

void loop() {
  if (ps5.isConnected()) {
    // Control Servos with Analog Sticks
    int servo1Angle = map(ps5.LStickY(), -128, 127, 0, 180); // Map left stick Y to Servo 1 angle
    int servo2Angle = map(ps5.RStickY(), -128, 127, 0, 180); // Map right stick Y to Servo 2 angle
    
    uint16_t pulse1 = SERVOMIN + ((SERVOMAX - SERVOMIN) * servo1Angle / 180);
    uint16_t pulse2 = SERVOMIN + ((SERVOMAX - SERVOMIN) * servo2Angle / 180);

    pwm.setPWM(SERVO1_CHANNEL, 0, pulse1);  // Move Servo 1 to angle
    pwm.setPWM(SERVO2_CHANNEL, 0, pulse2);  // Move Servo 2 to angle

    // Control Motors with D-pad
    if (ps5.UpLeft()) {
      moveMotorsForward();  // Move motors forward
      Serial.println("Motors Moving Forward");
    } else if (ps5.UpRight()) {
      moveMotorsBackward();  // Move motors backward
      Serial.println("Motors Moving Backward");
    } else {
      // Stop motors when no direction is pressed
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      Serial.println("Motors Stopped");
    }

    // Control specific motor directions with buttons
    if (ps5.Cross()) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      Serial.println("Motor A Forward");
    }
    if (ps5.Circle()) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      Serial.println("Motor A Backward");
    }
    if (ps5.Square()) {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      Serial.println("Motor B Forward");
    }
    if (ps5.Triangle()) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      Serial.println("Motor B Backward");
    }
  }
  
  delay(100);  // Avoid spamming Serial Monitor
}
