#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150
#define SERVOMAX  600
#define SERVO_FREQ 50

void setup() {
  Serial.begin(115200);
  Serial.println("2 channel Servo test!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
}

void loop() {
  // for rotating at same time
  for (int angle = 0; angle <= 180; angle++) {
    pwm.setPWM(0, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(4, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
    delay(15); 
  }

  for (int angle = 180; angle >= 0; angle--) {
    pwm.setPWM(0, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(4, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
    delay(15); 
  }
}
