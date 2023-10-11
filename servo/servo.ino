#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// #define SERVOMIN  110
// #define SERVOMAX  520
#define SERVOMIN  100
#define SERVOMAX  450
#define SERVO_FREQ 50
int satrting_servoAngle = ((SERVOMAX-SERVOMIN)/2)+SERVOMIN; 
int servoAngle;
void setup() {
  Serial.begin(115200);
  // Serial.println("2 channel Servo test!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  // pwm.setPWM(0, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
  // pwm.setPWM(4, 0, map(90, 180, 0, SERVOMIN, SERVOMAX));
  pwm.setPWM(0, 0,satrting_servoAngle);
  pwm.setPWM(4, 0,satrting_servoAngle);
  // delay(100);
  Serial.println("2 channel Servo test!");

}

void loop() {
  if (Serial.available()) {
    String angleStr = Serial.readStringUntil('\n');  // Read the incoming data until newline character
    int newAngle = angleStr.toInt();  // Convert the string to integer
    Serial.print("Setting servo time to: ");
    servoAngle = newAngle;
    Serial.println(servoAngle);
    }
  

  // Update the servo position
  // pwm.setPWM(3, 0, servoAngle);
  pwm.setPWM(0, 0, map(servoAngle, 0, 180, SERVOMIN, SERVOMAX));
  pwm.setPWM(4, 0, map(servoAngle, 180, 0, SERVOMIN, SERVOMAX));
  // delay(10);
  
}
